package frc.robot.subsystems.shooter.flywheels;

import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;

import static edu.wpi.first.units.Units.*;
import static frc.lib.generic.hardware.motor.MotorProperties.ControlMode.VOLTAGE;
import static frc.robot.subsystems.shooter.flywheels.FlywheelConstants.*;

public class Flywheel extends GenericSubsystem {
    public Command setFlywheelVelocity(double velocity) {
        return Commands.run(() -> {
            MASTER_LEFT_FLYWHEEL_MOTOR.setOutput(MotorProperties.ControlMode.VELOCITY, velocity);
        }, this);
    }

    public Command stop() {
        return Commands.runOnce(MASTER_LEFT_FLYWHEEL_MOTOR::stopMotor, this);
    }

    public double getFlywheelVelocity() {
        return MASTER_LEFT_FLYWHEEL_MOTOR.getSystemVelocity();
    }

    public double getFlywheelTargetVelocity() {
        return MASTER_LEFT_FLYWHEEL_MOTOR.getClosedLoopTarget();
    }

    public double getFlywheelVoltage() {
        return MASTER_LEFT_FLYWHEEL_MOTOR.getVoltage();
    }

    @Override
    public void periodic() {
        if (FLYWHEEL_MECHANISM == null) return;

        FLYWHEEL_MECHANISM.updateCurrentSpeed(getFlywheelVelocity());
        FLYWHEEL_MECHANISM.updateTargetSpeed(getFlywheelTargetVelocity());
    }

    @Override
    public SysIdRoutine.Config getSysIdConfig() {
        return SYSID_FLYWHEEL_CONFIG;
    }

    @Override
    public void sysIdDrive(double voltage) {
        MASTER_LEFT_FLYWHEEL_MOTOR.setOutput(VOLTAGE, voltage);
    }

    @Override
    public void sysIdUpdateLog(SysIdRoutineLog log) {
        log.motor("FLYWHEEL_MASTER_VELOCITY" + MASTER_LEFT_FLYWHEEL_MOTOR.getDeviceID())
                .voltage(Volts.of(MASTER_LEFT_FLYWHEEL_MOTOR.getVoltage()))
                .angularPosition(Rotations.of(MASTER_LEFT_FLYWHEEL_MOTOR.getSystemPosition()))
                .angularVelocity(RotationsPerSecond.of(MASTER_LEFT_FLYWHEEL_MOTOR.getSystemVelocity()));
    }
}