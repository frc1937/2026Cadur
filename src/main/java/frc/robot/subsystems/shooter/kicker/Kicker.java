package frc.robot.subsystems.shooter.kicker;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.characterization.FindMaxSpeedCommand;
import frc.lib.generic.hardware.motor.MotorProperties;

import static frc.lib.generic.hardware.motor.MotorProperties.ControlMode.VELOCITY;
import static frc.lib.generic.hardware.motor.MotorProperties.ControlMode.VOLTAGE;
import static frc.robot.RobotContainer.*;
import static frc.robot.subsystems.shooter.kicker.KickerConstants.KICKER_MOTOR;

public class Kicker extends GenericSubsystem {
    private final static double KICKER_MPS_TO_FLYWHEEL_MPS = 1.75;
    private final double KICKER_VOLTAGE = 12;

    public Command followState() {
        return run(() -> {
            switch (SHOOTER_STATES.getState()) {
                case IDLE, SHOOTING_PASSING_HUB_BLOCKED, NOTHING -> stopMotor();
                case SHOOTING_HUB -> {
                    if (SHOOTER_STATES.isReadyToShoot()) copyFlywheelSpeed();
                    else stopMotor();
                }
                case SHOOTING_PASSING -> copyFlywheelSpeed();
            }
        });
    }

    public Command findMaxVelocity() {
        return new FindMaxSpeedCommand(KICKER_MOTOR, this);
    }

    public Command copyFlywheel(double rps) {
        return new FunctionalCommand(
                () -> {},
                () -> KICKER_MOTOR.setOutput(VELOCITY, KICKER_MPS_TO_FLYWHEEL_MPS * rps),
                (interrupted) -> KICKER_MOTOR.stopMotor(),
                () -> false,
                this
        );
    }

    public Command run() {
        return new FunctionalCommand(
                () -> {},
                () -> KICKER_MOTOR.setOutput(VOLTAGE, KICKER_VOLTAGE),
                (interrupted) -> KICKER_MOTOR.stopMotor(),
                () -> false,
                this
        );
    }

    public Command stop() {
        return Commands.runOnce(KICKER_MOTOR::stopMotor, this);
    }

    public double getSystemVoltage() {
        return KICKER_MOTOR.getVoltage();
    }

    private void copyFlywheelSpeed() {
        KICKER_MOTOR.setOutput(VELOCITY, 3.5/2.0 * SHOOTING_CALCULATOR.getResults().flywheelRPS());
    }

    private void stopMotor() {
        KICKER_MOTOR.stopMotor();
    }

    private void setVoltage(double voltage) {
        KICKER_MOTOR.setOutput(MotorProperties.ControlMode.VOLTAGE, voltage);
    }
}