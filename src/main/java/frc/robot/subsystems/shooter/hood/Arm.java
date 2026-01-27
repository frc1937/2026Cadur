package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;
import static frc.lib.generic.hardware.motor.MotorProperties.ControlMode.VOLTAGE;
import static frc.robot.RobotContainer.TURRET;
import static frc.robot.subsystems.shooter.hood.ArmConstants.*;

public class Arm extends GenericSubsystem {
    public Command setArmPosition(double position) {
        return new FunctionalCommand(
                () -> {},
                () -> ARM_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, position),
                interrupt -> ARM_MOTOR.stopMotor(),
                () -> false,
                this
        );
    }

    public Rotation2d getCurrentArmPosition() {
        return Rotation2d.fromRotations(ARM_MOTOR.getSystemPosition());
    }

    public Rotation2d getTargetArmPosition() {
        return Rotation2d.fromRotations(ARM_MOTOR.getClosedLoopTarget());
    }

    public Command stopArm() {
        return Commands.runOnce(ARM_MOTOR::stopMotor, this);
    }

    public void printPose() {
        if (ARM_MECHANISM != null) {
            final Pose3d current3dPose = new Pose3d(new Translation3d(0, 0, 0.75), new Rotation3d(0, getCurrentArmPosition().getRadians(), TURRET.getCurrentTurretPosition().getRadians()));

            Logger.recordOutput("Components/ArmPose", current3dPose);

            ARM_MECHANISM.updateCurrentAngle(getCurrentArmPosition());
            ARM_MECHANISM.updateTargetAngle(getTargetArmPosition());
        }
    }

    @Override
    public SysIdRoutine.Config getSysIdConfig() {
        return SYSID_ARM_CONFIG;
    }

    @Override
    public void sysIdDrive(double voltage) {
        ARM_MOTOR.setOutput(VOLTAGE, voltage);
    }

    @Override
    public void sysIdUpdateLog(SysIdRoutineLog log) {
        log.motor("TURRET_PITCH_MOTOR" + ARM_MOTOR.getDeviceID())
                .voltage(Volts.of(ARM_MOTOR.getVoltage()))
                .angularPosition(Rotations.of(ARM_MOTOR.getSystemPosition()))
                .angularVelocity(RotationsPerSecond.of(ARM_MOTOR.getSystemVelocity()));
    }
}
