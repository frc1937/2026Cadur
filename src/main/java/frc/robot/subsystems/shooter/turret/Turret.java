package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.GenericSubsystem;
import frc.lib.math.TimeAdjustedTransform;
import frc.lib.generic.characterization.FindMaxSpeedCommand;
import frc.robot.subsystems.shooter.ShooterStates;
import frc.robot.subsystems.shooter.ShootingCalculator;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.math.MathUtil.inputModulus;
import static edu.wpi.first.math.geometry.Pose3d.kZero;
import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj.RobotController.getFPGATime;
import static frc.lib.generic.hardware.motor.MotorProperties.ControlMode.VOLTAGE;
import static frc.lib.util.flippable.Flippable.isRedAlliance;
import static frc.lib.util.flippable.FlippableUtils.flipAboutYAxis;
import static frc.robot.RobotContainer.*;
import static frc.robot.subsystems.shooter.ShooterStates.ShooterState.SHOOTING_PASSING;
import static frc.robot.subsystems.shooter.ShooterStates.ShooterState.SHOOTING_PASSING_HUB_BLOCKED;
import static frc.robot.subsystems.shooter.ShootingConstants.PHASE_DELAY_SECONDS;
import static frc.robot.subsystems.shooter.turret.TurretConstants.*;
import static frc.robot.utilities.FieldConstants.*;
import static java.lang.Math.abs;

public class Turret extends GenericSubsystem {
    private final TimeAdjustedTransform transformCalculator = new TimeAdjustedTransform(2.0, kZero.transformBy(ROBOT_TO_CENTER_TURRET), this::getSelfRelativePosition);

    public Command followState() {
        return run(() -> {
            switch (SHOOTER_STATES.getState()) {
                case IDLE -> trackPosition(HUB_TOP_POSITION.get().toTranslation2d());
                case SHOOTING_HUB, SHOOTING_HUB_KICKER_ACCELERATING -> setTargetPosition(getSOTMAngle().getRotations(), getSOTMVelocity(), TrackingMode.AGGRESSIVE);
                case SHOOTING_PASSING, SHOOTING_PASSING_HUB_BLOCKED -> trackPassing();
                case NOTHING -> {}
            }
        });
    }

    public Command testTurretAntiRotation() {
        return run(() -> {
            final Rotation2d setpoint = Rotation2d.kPi.minus(POSE_ESTIMATOR.getCurrentAngle());
            setTargetPosition(setpoint.getRotations(), getCounterRotationVelocity(), TrackingMode.PASSIVE);
        }).andThen(stopTurret());
    }

    public Command testTurret(double targetPosition, double targetVelocity) {
        return run(() -> {
            Logger.recordOutput("Turret/TargetPosition", targetPosition);
            Logger.recordOutput("Turret/TargetVelocity", targetVelocity);
            Logger.recordOutput("Turret/CurrentPosition", TURRET_MOTOR.getSystemPosition());
            Logger.recordOutput("Turret/CurrentVelocity", TURRET_MOTOR.getSystemVelocity());

            Logger.recordOutput("Turret/PositionError", TURRET_MOTOR.getSystemPosition() - targetPosition);
            Logger.recordOutput("Turret/VelocityError", TURRET_MOTOR.getSystemVelocity() - targetVelocity);

            setTargetPosition(targetPosition, targetVelocity, TrackingMode.AGGRESSIVE);
        });
    }

    public Command getMaxValues() {
        return new FindMaxSpeedCommand(TURRET_MOTOR, this);
    }

    public Command stopTurret() {
        return Commands.runOnce(TURRET_MOTOR::stopMotor, this);
    }

    @AutoLogOutput(key = "Turret/IsReadyToShoot")
    public boolean isReadyToShootPhysics() {
        final ShootingCalculator.ShootingParameters latestResults = SHOOTING_CALCULATOR.getResults();

        if (!latestResults.isValid())
            return false;

        final double targetAngleRotations = getSOTMAngle().getRotations();
        final double angleError = abs(inputModulus(targetAngleRotations - TURRET_MOTOR.getSystemPosition(), -0.5, 0.5));

        return angleError < TURRET_ANGLE_TOLERANCE_ROTATIONS;
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

    public double getTurretVelocity() {
        return TURRET_MOTOR.getSystemVelocity();
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
        final Pose2d turretPose = POSE_ESTIMATOR.predictFuturePose(PHASE_DELAY_SECONDS).transformBy(ROBOT_TO_CENTER_TURRET_2d);
        final Translation2d turretToTarget = targetPosition.minus(turretPose.getTranslation());
        final Rotation2d robotRelativeAngle = turretToTarget.getAngle().minus(turretPose.getRotation());

        setTargetPosition(robotRelativeAngle.getRotations(), getCounterRotationVelocity(), TrackingMode.PASSIVE);
    }

    private void setTargetPosition(double targetAngle, double targetVelocity, TrackingMode mode) {
        final double currentPosition = TURRET_MOTOR.getSystemPosition();
        final double optimizedTarget = calculateOptimalTarget(currentPosition, targetAngle, mode);

        final double constrainedTargetAngle = MathUtil.clamp(
                optimizedTarget,
                MIN_ANGLE_ROT,
                MAX_ANGLE_ROT
        );

        TURRET_MOTOR.setMovingOutput(constrainedTargetAngle, targetVelocity);
    }

    private static double calculateOptimalTarget(double currentPos, double desiredAngle, TrackingMode mode) {
        final double delta = inputModulus(desiredAngle - currentPos, -0.5, 0.5);
        final double direct = currentPos + delta;

        return mode.select(currentPos, direct, direct + 1.0, direct - 1.0, MIN_ANGLE_ROT, MAX_ANGLE_ROT);
    }

    private double getSOTMVelocity() {
        return getCounterRotationVelocity() + SHOOTING_CALCULATOR.getResults().turretVelocityRotPS();
    }

    private static Rotation2d getSOTMAngle() {
        final Rotation2d fieldRelativeAngle = SHOOTING_CALCULATOR.getResults().turretAngle();
        return fieldRelativeAngle.minus(POSE_ESTIMATOR.predictFuturePose(PHASE_DELAY_SECONDS).getRotation());
    }

    private void trackPassing() {
        final double turretToHubY = (POSE_ESTIMATOR.getPose().transformBy(ROBOT_TO_CENTER_TURRET_2d)
                .getY() - HUB_TOP_POSITION.get().getY());

        ShooterStates.ShooterState targetState = SHOOTING_PASSING;

        if (abs(turretToHubY) <= HUB_SIZE)
            targetState = SHOOTING_PASSING_HUB_BLOCKED;

        if (SHOOTER_STATES.getState() != targetState)
            CommandScheduler.getInstance().schedule(SHOOTER_STATES.setState(targetState));

        if (targetState == SHOOTING_PASSING_HUB_BLOCKED)
            return;

        Translation2d targetPosition = (turretToHubY > 0) ? RIGHT_PASSING_POINT : LEFT_PASSING_POINT;
        targetPosition = isRedAlliance() ? flipAboutYAxis(targetPosition) : targetPosition;

        trackPosition(targetPosition);
    }

    private void trackDriverStation() {
        final double robotY = POSE_ESTIMATOR.getPose().getY();
        trackPosition(new Translation2d(isRedAlliance() ? FIELD_LENGTH : 0, robotY));
    }

    private double getCounterRotationVelocity() {
        return -SWERVE.getOmegaFromGyroRps();
    }
}