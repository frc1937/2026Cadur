package frc.robot.subsystems.intake;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;

import static frc.robot.subsystems.intake.IntakeConstants.INTAKE_MOTOR;

public class Intake extends GenericSubsystem {
    public Command setIntakeVoltage(double voltage) {
        return Commands.run(() -> setVoltage(voltage), this);
    }

    public Command stop() {
        return Commands.runOnce(INTAKE_MOTOR::stopMotor, this);
    }

    public double getSystemVelocity() {
        return INTAKE_MOTOR.getSystemVelocity();
    }

    private void setVoltage(double voltage) {
        INTAKE_MOTOR.setOutput(MotorProperties.ControlMode.VOLTAGE, voltage);
    }
}