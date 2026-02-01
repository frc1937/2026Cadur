package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;
import frc.lib.util.commands.FindMaxSpeedCommand;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;
import static frc.lib.generic.hardware.motor.MotorProperties.ControlMode.VOLTAGE;
import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.subsystems.shooter.ShootingCalculator.MIN_DISTANCE;
import static frc.robot.subsystems.shooter.turret.TurretConstants.*;
import static frc.robot.utilities.FieldConstants.HUB_TOP_POSITION;

public class Turret extends GenericSubsystem {
    public Command trackHub() {
        return new FunctionalCommand(
                () -> {},
                () -> {
                    final Pose2d futurePose = POSE_ESTIMATOR.predictFuturePose(MIN_DISTANCE);

                    final Rotation2d fieldRelativeAngle = HUB_TOP_POSITION.get().toTranslation2d()
                            .minus(futurePose.getTranslation())
                            .getAngle();

                    final Rotation2d robotRelativeAngle = fieldRelativeAngle.minus(futurePose.getRotation());

                    final double constrainedTarget = MathUtil.clamp(
                            robotRelativeAngle.getRotations(),
                            MIN_ANGLE.getRotations(),
                            MAX_ANGLE.getRotations()
                    );

                    setTargetPosition(constrainedTarget);
                },
                interrupt -> {
                },
                () -> false,
                this
        );
    }

    public Command getMaxValues() {
        return new FindMaxSpeedCommand(TURRET_MOTOR, this);
    }

    public Command stop() {
        return Commands.runOnce(TURRET_MOTOR::stopMotor, this);
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
     * @Units in rotations
     */
    private void setTargetPosition(double targetPosition) {
        TURRET_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, targetPosition);
    }
}