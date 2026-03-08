package frc.robot.subsystems.shooter.kicker;


import edu.wpi.first.wpilibj2.command.*;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;
import frc.robot.subsystems.shooter.ShooterStates;

import static frc.lib.generic.hardware.motor.MotorProperties.ControlMode.VOLTAGE;
import static frc.robot.RobotContainer.SHOOTER_STATES;
import static frc.robot.subsystems.shooter.kicker.KickerConstants.KICKER_MOTOR;

public class Kicker extends GenericSubsystem {
    public Command followState(ShooterStates states) {
        return run(() -> {
            switch (states.getState()) {
                case IDLE -> stopMotor();
                case SHOOTING_HUB -> {
                    if (SHOOTER_STATES.isReadyToShoot()) setVoltage(10);
                    else stopMotor();
                }
                case SHOOTING_PASSING -> setVoltage(10);
            }
        });
    }

    public Command releaseBall() {
        return run(() -> setVoltage(4)).withTimeout(0.3).andThen(stop());
        //TODO: Make this stop after EXACTLY one ball.
    }

    public Command run() {
        return new FunctionalCommand(
                () -> {},
                () -> KICKER_MOTOR.setOutput(VOLTAGE, 10),
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

    private void stopMotor() {
        KICKER_MOTOR.stopMotor();
    }

    private void setVoltage(double voltage) {
        KICKER_MOTOR.setOutput(MotorProperties.ControlMode.VOLTAGE, voltage);
    }
}