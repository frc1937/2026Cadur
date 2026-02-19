package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;
import frc.lib.math.TimeAdjustedTransform;
import frc.lib.util.commands.FindMaxSpeedCommand;
import frc.robot.subsystems.shooter.ShootingCalculator;
import frc.robot.utilities.FieldConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.math.geometry.Pose3d.kZero;
import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj.RobotController.getFPGATime;
import static frc.lib.generic.hardware.motor.MotorProperties.ControlMode.VOLTAGE;
import static frc.lib.math.Conversions.radpsToRps;
import static frc.lib.util.flippable.Flippable.isRedAlliance;
import static frc.lib.util.flippable.FlippableUtils.flipAboutYAxis;
import static frc.robot.RobotContainer.*;
import static frc.robot.subsystems.shooter.ShootingConstants.PHASE_DELAY;
import static frc.robot.subsystems.shooter.turret.TurretConstants.*;
import static frc.robot.utilities.FieldConstants.*;
import static java.lang.Math.signum;

public class Turret extends GenericSubsystem {
    private final TimeAdjustedTransform transformCalculator = new TimeAdjustedTransform(2.0, kZero.transformBy(ROBOT_TO_CENTER_TURRET), this::getSelfRelativePosition);

    public Command trackPassingPoint() {
        return run(() -> {
            final Translation2d robot = POSE_ESTIMATOR.getPose().getTranslation();
            final Translation2d hubToRobot = robot.minus(HUB_TOP_POSITION.get().toTranslation2d());

            if (Math.abs(hubToRobot.getY()) <= FieldConstants.HUB_SIZE / 2) return;

            Translation2d targetPosition = (hubToRobot.getY() > 0) ? RIGHT_PASSING_POINT : LEFT_PASSING_POINT;
            targetPosition = isRedAlliance() ? flipAboutYAxis(targetPosition) : targetPosition;

            trackPosition(targetPosition);
        });
    }

    public Command trackHubIdly() {
        return run(() -> trackPosition(HUB_TOP_POSITION.get().toTranslation2d()));
    }

    public Command trackHubForSOTM() {
        return run(() -> setTargetPosition(getSOTMTargetAngle().getRotations(), computeSOTMFeedforward(), TrackingMode.AGGRESSIVE));
    }


    // TODO: Run on real robot! see if turret holds position when chassis is rotating. After tuning kV and kS. kA if needed
    public Command testTurretAntiRotation() {
        return run(() -> {
            final Rotation2d setpoint = Rotation2d.fromDegrees(0).minus(POSE_ESTIMATOR.getCurrentAngle());
            setTargetPosition(setpoint.getRotations(), getFeedforwardVoltage(getCounterRotationVelocity()), TrackingMode.AGGRESSIVE);
        }).andThen(stopTurret());
    }

    public Command getMaxValues() {
        return new FindMaxSpeedCommand(TURRET_MOTOR, this);
    }

    public Command stopTurret() {
        return Commands.runOnce(TURRET_MOTOR::stopMotor, this);
    }

    @AutoLogOutput(key = "Turret/IsReadyToShoot")
    public boolean isReadyToShoot() {
        final ShootingCalculator.ShootingParameters latestResults = SHOOTING_CALCULATOR.getResults();

        if (!latestResults.isValid()) return false;

        final double targetAngleRotations = getSOTMTargetAngle().getRotations();
        final double targetVelocityRps = getCounterRotationVelocity() + latestResults.turretVelocityRotPS();

        final double angleError = Math.abs(MathUtil.inputModulus(targetAngleRotations - TURRET_MOTOR.getSystemPosition(), -0.5, 0.5));

        return angleError < TURRET_ANGLE_TOLERANCE_ROTATIONS &&
                Math.abs(targetVelocityRps - TURRET_MOTOR.getSystemVelocity()) < TURRET_VELOCITY_TOLERANCE_RPS;
    }

    @Override
    public void periodic() {
        transformCalculator.update(getSelfRelativePosition(), getFPGATime() / 1e6, TURRET_MOTOR.getSystemVelocity());
    }

    public Transform3d getCameraTransform(double timestamp) {
        return transformCalculator.getRobotToCamera(timestamp, TURRET_CENTER_TO_CAMERA);
    }

    public Rotation2d getSelfRelativePosition() {
        return Rotation2d.fromRotations(TURRET_MOTOR.getSystemPosition());
    }

    public Rotation2d getTargetPosition() {
        return Rotation2d.fromRotations(TURRET_MOTOR.getClosedLoopTarget());
    }

    public void printPose() {
        if (TURRET_MECHANISM != null) {
            final Rotation2d currentTurretPosition = getSelfRelativePosition();
            final Rotation2d targetTurretPosition = getTargetPosition();
            final Pose3d current3dPose = new Pose3d(0, 0, 0.5, new Rotation3d(0, 0, currentTurretPosition.getRadians()));

            Logger.recordOutput("Components/TurretPose", current3dPose);

            TURRET_MECHANISM.updateCurrentAngle(currentTurretPosition);
            TURRET_MECHANISM.updateTargetAngle(targetTurretPosition);
        }
    }

    @Override
    public SysIdRoutine.Config getSysIdConfig() {
        return SYSID_TURRET_CONFIG;
    }

    @Override
    public void sysIdDrive(double voltage) {
        TURRET_MOTOR.setOutput(VOLTAGE, voltage);
    }

    @Override
    public void sysIdUpdateLog(SysIdRoutineLog log) {
        log.motor("TURRET_MOTOR_YAW" + TURRET_MOTOR.getDeviceID())
                .voltage(Volts.of(TURRET_MOTOR.getVoltage()))
                .angularPosition(Rotations.of(TURRET_MOTOR.getSystemPosition()))
                .angularVelocity(RotationsPerSecond.of(TURRET_MOTOR.getSystemVelocity()));
    }


    private void trackPosition(Translation2d targetPosition) {
        final Pose2d robot = POSE_ESTIMATOR.getPose();
        final Translation2d robotToTarget = targetPosition.minus(robot.getTranslation());
        final Rotation2d robotRelativeAngle = robotToTarget.getAngle().minus(robot.getRotation());

        setTargetPosition(robotRelativeAngle.getRotations(), getFeedforwardVoltage(getCounterRotationVelocity()), TrackingMode.PASSIVE);
    }


    /**
     * Clamps target position within turret limits.
     *
     * @Units in rotations.
     */
    private void setTargetPosition(double targetAngle, double feedforward, TrackingMode mode) {
        final double currentPosition = TURRET_MOTOR.getSystemPosition();
        final double optimizedTarget = calculateOptimalTarget(currentPosition, targetAngle, mode);

        final double constrainedTargetAngle = MathUtil.clamp(
                optimizedTarget,
                MIN_ANGLE.getRotations(),
                MAX_ANGLE.getRotations()
        );

        TURRET_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, constrainedTargetAngle, feedforward);
    }

    private static double calculateOptimalTarget(double currentPos, double desiredAngle, TrackingMode mode) {
        final double delta = MathUtil.inputModulus(desiredAngle - currentPos, -0.5, 0.5);
        final double direct = currentPos + delta;

        return mode.select(currentPos, direct, direct + 1.0, direct - 1.0, MIN_ANGLE.getRotations(), MAX_ANGLE.getRotations());
    }

    /**
     * Compensates for robot rotation and turret tracking velocity.
     *
     * @return feedforward voltage to apply, using motor kV and kS values.
     */
    private static double computeSOTMFeedforward() {
        final double trackingVelocity = SHOOTING_CALCULATOR.getResults().turretVelocityRotPS();
        final double totalTargetVel = getCounterRotationVelocity() + trackingVelocity;

        return getFeedforwardVoltage(totalTargetVel);
    }

    private static Rotation2d getSOTMTargetAngle() {
        final Rotation2d fieldRelativeAngle = SHOOTING_CALCULATOR.getResults().turretAngle();
        return fieldRelativeAngle.minus(POSE_ESTIMATOR.predictFuturePose(PHASE_DELAY).getRotation());
    }

    private static double getCounterRotationVelocity() {
        return radpsToRps(-SWERVE.getRobotRelativeVelocity().omegaRadiansPerSecond);
    }

    private static double getFeedforwardVoltage(double targetVelocity) {
        return (TURRET_MOTOR.getConfig().slot.kV * targetVelocity) + (TURRET_MOTOR.getConfig().slot.kS * signum(targetVelocity));
    }
}