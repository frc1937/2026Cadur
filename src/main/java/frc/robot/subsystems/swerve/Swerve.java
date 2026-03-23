package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.OdometryThread;
import frc.lib.math.Optimizations;
import frc.robot.RobotContainer;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static frc.lib.math.Optimizations.isColliding;
import static frc.robot.GlobalConstants.PERIODIC_TIME_SEC;
import static frc.robot.RobotContainer.*;
import static frc.robot.subsystems.swerve.SwerveConstants.*;
import static frc.robot.subsystems.swerve.SwerveModuleConstants.MODULES;
import static frc.robot.utilities.PathingConstants.ROBOT_CONFIG;
import static java.lang.Math.abs;

public class Swerve extends GenericSubsystem {
    private final Timer xTimer = new Timer();
    private boolean hasXd = false;

    private double lastTimestamp = Timer.getFPGATimestamp();

    private final SwerveModulePosition[][] cachedWheelPositions;
    private final Rotation2d[] cachedGyroRotations = new Rotation2d[MAX_ODOMETRY_UPDATES];

    private ChassisSpeeds previousVelocity = new ChassisSpeeds();
    private ChassisSpeeds currentVelocity = new ChassisSpeeds();

    private static final int MAX_ODOMETRY_UPDATES = 100;

    public Swerve() {
        cachedWheelPositions = new SwerveModulePosition[MAX_ODOMETRY_UPDATES][MODULES.length];

        for (int i = 0; i < MAX_ODOMETRY_UPDATES; i++) {
            for (int j = 0; j < MODULES.length; j++) {
                cachedWheelPositions[i][j] = new SwerveModulePosition();
            }
        }
    }

    public boolean isAtPose(Pose2d target, double allowedDistanceFromTargetMeters, double allowedRotationalErrorDegrees) {
        Logger.recordOutput("Swerve/DistanceError", POSE_ESTIMATOR.getPose().getTranslation().getDistance(target.getTranslation()));
        Logger.recordOutput("Swerve/RotationError", abs(POSE_ESTIMATOR.getPose().getRotation().minus(target.getRotation()).getDegrees()));

        return POSE_ESTIMATOR.getPose().getTranslation().getDistance(target.getTranslation()) < allowedDistanceFromTargetMeters &&
                abs(POSE_ESTIMATOR.getPose().getRotation().minus(target.getRotation()).getDegrees()) < allowedRotationalErrorDegrees;
    }

    @Override
    public SysIdRoutine.Config getSysIdConfig() {
        return SYSID_DRIVE_CONFIG;
    }

    @Override
    public void sysIdDrive(double voltage) {
        for (SwerveModule module : MODULES) {
            module.runDriveMotorForCharacterization(voltage);
        }
    }

    @Override
    public void sysIdUpdateLog(SysIdRoutineLog log) {
        MODULES[0].logForSysId(log);
    }

    public void setGyroHeading(Rotation2d heading) {
        GYRO.setGyroYaw(heading.getRotations());
    }

    public double getGyroHeading() {
        return GYRO.getYawRotations();
    }

    public double getOmegaFromGyroRps() {
        return GYRO.getGyroYawRate();
    }

    @AutoLogOutput(key = "Swerve/velocity")
    public ChassisSpeeds getRobotRelativeVelocity() {
        return currentVelocity;
    }

    @AutoLogOutput(key = "Swerve/acceleration")
    public ChassisSpeeds getRobotRelativeAcceleration() {
        return currentVelocity.minus(previousVelocity).div(PERIODIC_TIME_SEC);
    }

    public ChassisSpeeds getFieldRelativeVelocity() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotRelativeVelocity(), POSE_ESTIMATOR.getPose().getRotation());
    }

    public void runDriveMotorWheelCharacterization(double voltage) {
        for (SwerveModule module : MODULES)
            module.runDriveMotorForCharacterization(voltage);
    }

    public double[] getDriveWheelPositionsRadians() {
        final double[] driveWheelPositions = new double[MODULES.length];

        for (int i = 0; i < MODULES.length; i++)
            driveWheelPositions[i] = MODULES[i].getDriveWheelPositionRadians();

        return driveWheelPositions;
    }

    @Override
    public void periodic() {
        final double[] odometryUpdatesYawRotations = GYRO.getInputs().threadGyroYawRotations;
        final double[] timestamps = OdometryThread.getInstance().getLatestTimestamps();

        final int odometryUpdates = odometryUpdatesYawRotations.length;

        if (odometryUpdates == 0 || timestamps.length == 0) return;

        final int count = Math.min(odometryUpdates, MAX_ODOMETRY_UPDATES);

        for (int i = 0; i < count; i++) {
            fillSwerveWheelPositions(cachedWheelPositions[i], i);
            cachedGyroRotations[i] = Rotation2d.fromRotations(odometryUpdatesYawRotations[i]);
        }

        previousVelocity = currentVelocity;
        currentVelocity = SWERVE_KINEMATICS.toChassisSpeeds(getModuleStates());

//        if (isColliding()) //TODO: Use gyro instead. this consumes 8% of CPU!!!!
//            return;

        POSE_ESTIMATOR.updateFromOdometry(
                cachedWheelPositions,
                cachedGyroRotations,
                timestamps,
                count
        );
    }

    public void driveRobotRelative(ChassisSpeeds chassisSpeeds, boolean shouldUseClosedLoop) {
        final SwerveModuleState[] swerveModuleStates = SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, ROBOT_CONFIG.moduleConfig.maxDriveVelocityMPS);
        chassisSpeeds = discretize(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, ROBOT_CONFIG.moduleConfig.maxDriveVelocityMPS);

        if (Optimizations.isStill(chassisSpeeds)) {
            stop();

            if (!xTimer.isRunning())
                xTimer.restart();

            if (xTimer.get() > 1.1 && !hasXd) {
                MODULES[0].setTargetState(MODULE_ORIENTATION_LEFT, false);
                MODULES[1].setTargetState(MODULE_ORIENTATION_RIGHT, false);
                MODULES[2].setTargetState(MODULE_ORIENTATION_RIGHT, false);
                MODULES[3].setTargetState(MODULE_ORIENTATION_LEFT, false);
                hasXd = true;
            }

            return;
        }

        xTimer.stop();
        hasXd = false;

        for (int i = 0; i < MODULES.length; i++)
            MODULES[i].setTargetState(swerveModuleStates[i], shouldUseClosedLoop);
    }

    public void rotateToTargetFromPresetGoal() {
        driveFieldRelative(
                0,
                0,
                SWERVE_ROTATION_CONTROLLER.calculate(POSE_ESTIMATOR.getPose().getRotation().getDegrees()),
                true
        );
    }

    protected void driveOpenLoop(double xPower, double yPower, double thetaPower, boolean robotCentric) {
        if (robotCentric)
            driveRobotRelative(xPower, yPower, thetaPower, false);
        else
            driveFieldRelative(xPower, yPower, thetaPower, false);
    }

    protected void driveWithTarget(double xPower, double yPower, boolean robotCentric) {
        final Rotation2d currentAngle = POSE_ESTIMATOR.getPose().getRotation();

        final double controllerOutput = Units.degreesToRadians(SWERVE_ROTATION_CONTROLLER.calculate(currentAngle.getDegrees()));

        if (robotCentric)
            driveRobotRelative(xPower, yPower, controllerOutput, false);
        else
            driveFieldRelative(xPower, yPower, controllerOutput, false);
    }

    protected double getOmegaToTarget(double targetRotations) {
        final double current = SWERVE.getGyroHeading();
        final double omegaOutput = SWERVE_ROTATION_PID.calculate(current, targetRotations);

        return SWERVE_ROTATION_PID.atSetpoint() ? 0 : omegaOutput;
    }


    protected void driveToPosePID(Pose2d target) {
        final Pose2d currentPose = POSE_ESTIMATOR.getPose();

        driveFieldRelative(
                PID_TRANSLATION_X_CONTROLLER.calculate(
                        currentPose.getX(),
                        target.getX()),
                PID_TRANSLATION_Y_CONTROLLER.calculate(
                        currentPose.getY(),
                        target.getY()),
                SWERVE_ROTATION_CONTROLLER.calculate(currentPose.getRotation().getDegrees()),
                true
        );
    }

    protected void driveFieldRelative(double xPower, double yPower, double thetaPower, boolean shouldUseClosedLoop) {
        ChassisSpeeds speeds = powerSpeedsToChassisSpeeds(new ChassisSpeeds(xPower, yPower, thetaPower));
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, RobotContainer.POSE_ESTIMATOR.getPose().getRotation());

        driveRobotRelative(speeds, shouldUseClosedLoop);
    }

    public void driveRobotRelative(double xPower, double yPower, double thetaPower, boolean shouldUseClosedLoop) {
        final ChassisSpeeds speeds = powerSpeedsToChassisSpeeds(new ChassisSpeeds(xPower, yPower, thetaPower));
        driveRobotRelative(speeds, shouldUseClosedLoop);
    }

    protected void resetRotationController() {
        SWERVE_ROTATION_CONTROLLER.reset(POSE_ESTIMATOR.getPose().getRotation().getDegrees(), getFieldRelativeVelocity().omegaRadiansPerSecond);
    }

    protected void setTargetRotation(Rotation2d target) {
        SWERVE_ROTATION_CONTROLLER.setGoal(target.getDegrees());
    }

    protected void fillSwerveWheelPositions(SwerveModulePosition[] positions, int odometryUpdateIndex) {
        for (int i = 0; i < MODULES.length; i++) {
            MODULES[i].fillOdometryPosition(positions[i], odometryUpdateIndex);
        }
    }

    protected ChassisSpeeds powerSpeedsToChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        return new ChassisSpeeds(
                chassisSpeeds.vxMetersPerSecond * MAX_SPEED_MPS,
                chassisSpeeds.vyMetersPerSecond * MAX_SPEED_MPS,
                chassisSpeeds.omegaRadiansPerSecond
        );
    }

    @AutoLogOutput(key = "Swerve/CurrentStates")
    public SwerveModuleState[] getModuleStates() {
        final SwerveModuleState[] states = new SwerveModuleState[MODULES.length];

        for (int i = 0; i < MODULES.length; i++)
            states[i] = MODULES[i].getCurrentState();

        return states;
    }

    @AutoLogOutput(key = "Swerve/TargetStates")
    @SuppressWarnings("unused")
    protected SwerveModuleState[] getModuleTargetStates() {
        final SwerveModuleState[] states = new SwerveModuleState[MODULES.length];

        for (int i = 0; i < MODULES.length; i++)
            states[i] = MODULES[i].getTargetState();

        return states;
    }

    public void stop() {
        for (SwerveModule currentModule : MODULES)
            currentModule.stop();
    }

    public double getTotalCurrent() {
        double total = 0;

        for (SwerveModule module : MODULES) {
            total += module.getCurrent();
        }

        return total;
    }

    /**
     * When the robot drives while rotating it skews a bit to the side.
     * This should fix the chassis speeds, so they won't make the robot skew while rotating.
     *
     * @param chassisSpeeds The chassis speeds to fix skewing for
     * @return the fixed speeds
     */
    protected ChassisSpeeds discretize(ChassisSpeeds chassisSpeeds) {
        final double currentTimestamp = Timer.getFPGATimestamp();
        final double difference = currentTimestamp - lastTimestamp;

        lastTimestamp = currentTimestamp;

        return ChassisSpeeds.discretize(chassisSpeeds, difference);
    }
}
