package frc.robot.subsystems.revolver;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;
import frc.robot.subsystems.shooter.ShooterStates;

import static frc.robot.RobotContainer.SHOOTER_STATES;
import static frc.robot.subsystems.revolver.RevolverConstants.REVOLVER_MOTOR;

public class Revolver extends GenericSubsystem {
    public Command followState(ShooterStates states) {
        return run(() -> {
            switch (states.getState()) {
                case IDLE -> stopMotor();
                case SHOOTING_HUB -> {
                    if (SHOOTER_STATES.isReadyToShoot()) setVoltage(8);
                    else stopMotor();
                }
                case SHOOTING_PASSING -> setVoltage(10);
            }
        });
    }

    public Command enableRevolver() {
        return new FunctionalCommand(
                () -> {},
                () -> setVoltage(8),
                (interrupt) -> REVOLVER_MOTOR.stopMotor(),
                () -> false,
                this
        );
    }

    public Command stop() {
        return Commands.runOnce(REVOLVER_MOTOR::stopMotor, this);
    }

    public double getSystemVoltage() {
        return REVOLVER_MOTOR.getVoltage();
    }

    private void stopMotor() {
        REVOLVER_MOTOR.stopMotor();
    }

    private void setVoltage(double voltage) {
        REVOLVER_MOTOR.setOutput(MotorProperties.ControlMode.VOLTAGE, voltage);
    }
}