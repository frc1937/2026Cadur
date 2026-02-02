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
import org.littletonrobotics.junction.Logger;

import java.util.Optional;

import static edu.wpi.first.math.interpolation.TimeInterpolatableBuffer.createBuffer;
import static edu.wpi.first.units.Units.*;
import static frc.lib.generic.hardware.motor.MotorProperties.ControlMode.VOLTAGE;
import static frc.robot.RobotContainer.*;
import static frc.robot.subsystems.shooter.turret.TurretConstants.*;

public class Turret extends GenericSubsystem {
    private final TimeInterpolatableBuffer<Rotation2d> turretAngleBuffer = createBuffer(2.0);

    public Command trackHub() {
        return new RunCommand(
                () -> {
                    final Rotation2d fieldRelativeAngle = SHOOTING_CALCULATOR.getParameters().turretAngle();
                    final Rotation2d robotRelativeAngle = fieldRelativeAngle.minus(POSE_ESTIMATOR.getPose().getRotation());
                    //TODO: May be beneficial to include FUTURE pose in here to account for latency. requires Testing

                    final double counterRotationFF = -SWERVE.getRobotRelativeVelocity().omegaRadiansPerSecond * COUNTER_ROTATION_FF;

                    setTargetPosition(robotRelativeAngle.getRotations(), counterRotationFF);
                },

                this
        );
    }

    public Command getMaxValues() {
        return new FindMaxSpeedCommand(TURRET_MOTOR, this);
    }

    public Command stop() {
        return Commands.runOnce(TURRET_MOTOR::stopMotor, this);
    }

    public boolean isAtGoal() {
        return TURRET_MOTOR.isAtPositionSetpoint();
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
}