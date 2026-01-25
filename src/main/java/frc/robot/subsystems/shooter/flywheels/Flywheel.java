package frc.robot.subsystems.shooter.flywheels;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;

import static frc.robot.subsystems.shooter.flywheels.FlywheelConstants.FLYWHEEL_MECHANISM;
import static frc.robot.subsystems.shooter.flywheels.FlywheelConstants.FLYWHEEL_MOTOR;

public class Flywheel extends GenericSubsystem {
    public Command setFlywheelVelocity(double velocity) {
        return Commands.run(() -> FLYWHEEL_MOTOR.setOutput(MotorProperties.ControlMode.VELOCITY, velocity), this);
    }

    public Command stop() {
        return Commands.runOnce(FLYWHEEL_MOTOR::stopMotor, this);
    }

    public double getCurrentVelocity() {
        return FLYWHEEL_MOTOR.getSystemVelocity();
    }

    public double getTargetVelocity() {
        return FLYWHEEL_MOTOR.getClosedLoopTarget();
    }

    public double getCurrentVoltage() {
        return FLYWHEEL_MOTOR.getVoltage();
    }

    @Override
    public void periodic() {
        if (FLYWHEEL_MECHANISM != null) {
            FLYWHEEL_MECHANISM.updateCurrentSpeed(getCurrentVelocity());
            FLYWHEEL_MECHANISM.updateTargetSpeed(getTargetVelocity());
        }
    }
}