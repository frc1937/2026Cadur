package frc.robot.subsystems.shooter.kicker;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.characterization.FindMaxSpeedCommand;
import frc.lib.generic.hardware.motor.MotorProperties;

import static frc.lib.generic.hardware.motor.MotorProperties.ControlMode.VELOCITY;
import static frc.lib.generic.hardware.motor.MotorProperties.ControlMode.VOLTAGE;
import static frc.robot.RobotContainer.SHOOTER_STATES;
import static frc.robot.subsystems.shooter.kicker.KickerConstants.KICKER_MOTOR;

public class Kicker extends GenericSubsystem {
    private final double KICKER_VOLTAGE = 12;

    public Command followState() {
        return run(() -> {
            switch (SHOOTER_STATES.getState()) {
                case IDLE, SHOOTING_PASSING_HUB_BLOCKED, NOTHING -> stopMotor();
                case SHOOTING_HUB -> {
                    if (SHOOTER_STATES.isReadyToShoot()) setAsFlywheel();
                    else stopMotor();
                }
                case SHOOTING_PASSING -> setAsFlywheel();
            }
        });
    }

    public Command findMaxVelocity() {
        return new FindMaxSpeedCommand(KICKER_MOTOR, this);
    }

    public Command setAtRPS(double rps) {
        return new FunctionalCommand(
                () -> {},
                () -> KICKER_MOTOR.setOutput(VELOCITY, rps),
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

    private void setAsFlywheel() {
//        KICKER_MOTOR.setOutput(VELOCITY, SHOOTING_CALCULATOR.getResults().flywheelRPS());
        KICKER_MOTOR.setOutput(VOLTAGE, KICKER_VOLTAGE);
    }

    private void stopMotor() {
        KICKER_MOTOR.stopMotor();
    }

    private void setVoltage(double voltage) {
        KICKER_MOTOR.setOutput(MotorProperties.ControlMode.VOLTAGE, voltage);
    }
}