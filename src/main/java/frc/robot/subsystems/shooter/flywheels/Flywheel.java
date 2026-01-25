package frc.robot.subsystems.shooter.flywheels;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;

import static frc.robot.subsystems.shooter.flywheels.FlywheelConstants.*;

public class Flywheel extends GenericSubsystem {
    public Command setFlywheelVelocity(double velocity) {
        return Commands.run(() -> {
            LEFT_FLYWHEEL_MOTOR.setOutput(MotorProperties.ControlMode.VELOCITY, velocity);
            RIGHT_FLYWHEEL_MOTOR.setOutput(MotorProperties.ControlMode.VELOCITY, velocity);
        }, this);
    }

    public Command stop() {
        return Commands.runOnce(() -> {
            LEFT_FLYWHEEL_MOTOR.stopMotor();
            RIGHT_FLYWHEEL_MOTOR.stopMotor();
        }, this);
    }

    public double getLeftFlywheelVelocity() {
        return LEFT_FLYWHEEL_MOTOR.getSystemVelocity();
    }

    public double getRightFlywheelVelocity() {
        return RIGHT_FLYWHEEL_MOTOR.getSystemVelocity();
    }

    public double getLeftFlywheelTargetVelocity() {
        return LEFT_FLYWHEEL_MOTOR.getClosedLoopTarget();
    }

    public double getRightFlywheelTargetVelocity() {
        return RIGHT_FLYWHEEL_MOTOR.getClosedLoopTarget();
    }

    public double getLeftFlywheelVoltage() {
        return LEFT_FLYWHEEL_MOTOR.getVoltage();
    }

    public double getRightFlywheelVoltage() {
        return RIGHT_FLYWHEEL_MOTOR.getVoltage();
    }

    @Override
    public void periodic() {
        if (LEFT_FLYWHEEL_MECHANISM == null || RIGHT_FLYWHEEL_MECHANISM == null) return;

        LEFT_FLYWHEEL_MECHANISM.updateCurrentSpeed(getLeftFlywheelVelocity());
        LEFT_FLYWHEEL_MECHANISM.updateTargetSpeed(getLeftFlywheelTargetVelocity());

        RIGHT_FLYWHEEL_MECHANISM.updateCurrentSpeed(getRightFlywheelVelocity());
        RIGHT_FLYWHEEL_MECHANISM.updateTargetSpeed(getRightFlywheelTargetVelocity());
    }
}