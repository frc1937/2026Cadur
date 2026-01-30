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
import static frc.robot.subsystems.shooter.hood.HoodConstants.*;

public class Hood extends GenericSubsystem {
    public Command setHoodPosition(double position) {
        return new FunctionalCommand(
                () -> {},
                () -> HOOD_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, position),
                interrupt -> HOOD_MOTOR.stopMotor(),
                () -> false,
                this
        );
    }

    public Rotation2d getCurrentPosition() {
        return Rotation2d.fromRotations(HOOD_MOTOR.getSystemPosition());
    }

    public Rotation2d getTargetPosition() {
        return Rotation2d.fromRotations(HOOD_MOTOR.getClosedLoopTarget());
    }

    public Command stopHood() {
        return Commands.runOnce(HOOD_MOTOR::stopMotor, this);
    }

    public void printPose() {
        if (HOOD_MECHANISM != null) {
            final Pose3d current3dPose = new Pose3d(new Translation3d(0, 0, 0.45), new Rotation3d(0, getCurrentPosition().getRadians(), TURRET.getCurrentPosition().getRadians()));

            Logger.recordOutput("Components/HoodPose", current3dPose);

            HOOD_MECHANISM.updateCurrentAngle(getCurrentPosition());
            HOOD_MECHANISM.updateTargetAngle(getTargetPosition());
        }
    }

    @Override
    public SysIdRoutine.Config getSysIdConfig() {
        return SYSID_HOOD_CONFIG;
    }

    @Override
    public void sysIdDrive(double voltage) {
        HOOD_MOTOR.setOutput(VOLTAGE, voltage);
    }

    @Override
    public void sysIdUpdateLog(SysIdRoutineLog log) {
        log.motor("TURRET_PITCH_MOTOR" + HOOD_MOTOR.getDeviceID())
                .voltage(Volts.of(HOOD_MOTOR.getVoltage()))
                .angularPosition(Rotations.of(HOOD_MOTOR.getSystemPosition()))
                .angularVelocity(RotationsPerSecond.of(HOOD_MOTOR.getSystemVelocity()));
    }
}
