package frc.robot.subsystems.shooter.kicker;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;

import static frc.robot.subsystems.shooter.kicker.KickerConstants.KICKER_MOTOR;

public class Kicker extends GenericSubsystem {
    public Command releaseBall() {
        return run(() -> setVoltage(4)).withTimeout(0.3).andThen(stop());
        //TODO: Make this stop after EXACTLY one ball.
    }

    public Command stop() {
        return Commands.runOnce(KICKER_MOTOR::stopMotor, this);
    }

    public double getSystemVoltage() {
        return KICKER_MOTOR.getVoltage();
    }

    private void setVoltage(double voltage) {
        KICKER_MOTOR.setOutput(MotorProperties.ControlMode.VOLTAGE, voltage);
    }
}