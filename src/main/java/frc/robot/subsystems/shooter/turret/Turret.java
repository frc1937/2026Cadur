package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;
import frc.lib.util.commands.FindMaxSpeedCommand;
import frc.robot.subsystems.shooter.ShootingCalculator;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;

import static edu.wpi.first.math.interpolation.TimeInterpolatableBuffer.createBuffer;
import static edu.wpi.first.units.Units.*;
import static frc.lib.generic.hardware.motor.MotorProperties.ControlMode.VOLTAGE;
import static frc.lib.math.Conversions.radpsToRps;
import static frc.robot.RobotContainer.*;
import static frc.robot.subsystems.shooter.ShootingCalculator.PHASE_DELAY;
import static frc.robot.subsystems.shooter.turret.TurretConstants.*;
import static java.lang.Math.signum;

public class Turret extends GenericSubsystem {
    private final TimeInterpolatableBuffer<Rotation2d> turretAngleBuffer = createBuffer(2.0);

    public Command trackHub() {
        return new RunCommand(
                () -> {
                    final Rotation2d fieldRelativeAngle = SHOOTING_CALCULATOR.getResults().turretAngle();
                    final Rotation2d robotRelativeAngle = fieldRelativeAngle.minus(POSE_ESTIMATOR.predictFuturePose(PHASE_DELAY).getRotation());

                    setTargetPosition(robotRelativeAngle.getRotations(), compensateForRotationAndTrackingFF());
                },
                this
        );
    }

    // TODO: Run on real robot! see if turret holds position when chassis is rotating. After tuning kV and kS. kA if needed
    public Command testTurretAntiRotation() {
        return Commands.run(
                () -> {
                    double antiRotationVelocity = -radpsToRps(SWERVE.getRobotRelativeVelocity().omegaRadiansPerSecond);
                    double feedforward =
                            (TURRET_MOTOR.getConfig().slot.kV * antiRotationVelocity) +
                            (TURRET_MOTOR.getConfig().slot.kS * signum(antiRotationVelocity));

                    Rotation2d setpoint = Rotation2d.fromDegrees(0).minus(POSE_ESTIMATOR.getCurrentAngle());

                    setTargetPosition(setpoint.getRotations(), feedforward);
                },
                this
        ).andThen(stopTurret());
    }

    public Command getMaxValues() {
        return new FindMaxSpeedCommand(TURRET_MOTOR, this);
    }

    public Command stopTurret() {
        return Commands.runOnce(TURRET_MOTOR::stopMotor, this);
    }

    public boolean isReadyToShoot() {
        final ShootingCalculator.ShootingParameters latestResults = SHOOTING_CALCULATOR.getResults();

        if (!latestResults.isValid()) return false;

        final double targetAngleRotations = latestResults.turretAngle().minus(POSE_ESTIMATOR.getCurrentAngle()).getRotations();
        final double targetVelocityRps = radpsToRps(-SWERVE.getRobotRelativeVelocity().omegaRadiansPerSecond) + latestResults.turretVelocityRotPS();

        return
                Math.abs(targetAngleRotations - TURRET_MOTOR.getSystemPosition()) < TURRET_ANGLE_TOLERANCE_ROTATIONS &&
                Math.abs(targetVelocityRps - TURRET_MOTOR.getSystemVelocity()) < TURRET_VELOCITY_TOLERANCE_RPS;
    }

    public double getTurretVelocityRadiansPerSec() {
        return TURRET_MOTOR.getSystemVelocity() * Math.PI * 2;
    }

    @Override
    public void periodic() {
        turretAngleBuffer.addSample(Timer.getTimestamp(), getCurrentPosition());
    }

    //todo: use with camera latency compensation
    public Optional<Rotation2d> getTurretAngle(double timestamp) {
        return turretAngleBuffer.getSample(timestamp);
    }

    public Rotation2d getCurrentPosition() {
        return Rotation2d.fromRotations(TURRET_MOTOR.getSystemPosition());
    }

    public Rotation2d getTargetPosition() {
        return Rotation2d.fromRotations(TURRET_MOTOR.getClosedLoopTarget());
    }

    public void printPose() {
        if (TURRET_MECHANISM != null) {
            final Rotation2d currentTurretPosition = getCurrentPosition();
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

    /**
     * Clamps target position within turret limits.
     *
     * @Units in rotations.
     */
    private void setTargetPosition(double targetAngle, double feedforward) {
        final double constrainedTargetAngle = MathUtil.clamp(
                targetAngle,
                MIN_ANGLE.getRotations(),
                MAX_ANGLE.getRotations()
        );

        TURRET_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, constrainedTargetAngle, feedforward);
    }

    /**
     * Compensates for robot rotation and turret tracking velocity.
     *
     * @return feedforward voltage to apply, using motor kV and kS values.
     */
    private static double compensateForRotationAndTrackingFF() {
        final double counterRotationVelocity = radpsToRps(-SWERVE.getRobotRelativeVelocity().omegaRadiansPerSecond); //well tuned kV and kS should handle this well
        final double trackingVelocity = SHOOTING_CALCULATOR.getResults().turretVelocityRotPS();

        final double totalTargetVel = counterRotationVelocity + trackingVelocity;

        return (TURRET_MOTOR.getConfig().slot.kV * totalTargetVel) + (TURRET_MOTOR.getConfig().slot.kS * signum(totalTargetVel));
    }
}