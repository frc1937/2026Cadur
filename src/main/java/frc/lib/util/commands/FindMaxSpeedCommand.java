package frc.lib.util.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.Motor;

import static frc.lib.generic.hardware.motor.MotorProperties.ControlMode.VOLTAGE;

public class FindMaxSpeedCommand extends Command {
    private double maxVelocity = 0, maxAcceleration = 0;

    private final Motor motor;

    public FindMaxSpeedCommand(Motor motor, GenericSubsystem subsystem) {
        this.motor = motor;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        System.out.println("USING MOTOR " + motor.getName() + " FOR MAX SPEED CHARACTERIZATION.. BEWARE!");
    }

    @Override
    public void execute() {
        motor.setOutput(VOLTAGE, 12.0);

        maxVelocity = Math.max(maxVelocity, Math.abs(motor.getSystemVelocity()));
        maxAcceleration = Math.max(maxAcceleration, Math.abs(motor.getSystemAcceleration()));
    }

    @Override
    public void end(boolean interrupted) {
        motor.stopMotor();
        System.out.printf("--- RESULTS ---\nVelocity: %.5f RPS\nAcceleration: %.5f RPS/s\nSuggested kV: %.4f\n", maxVelocity, maxAcceleration, 12.0 / maxVelocity);
    }
}