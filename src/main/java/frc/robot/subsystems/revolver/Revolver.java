package frc.robot.subsystems.revolver;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;

import static frc.robot.subsystems.revolver.RevolverConstants.REVOLVER_MOTOR;

public class Revolver extends GenericSubsystem {
    public Command setRevolverVoltage(double voltage) {
        return Commands.run(() -> setVoltage(voltage), this);
    }

    public Command stop() {
        return Commands.runOnce(REVOLVER_MOTOR::stopMotor, this);
    }

    public double getSystemVelocity() {
        return REVOLVER_MOTOR.getSystemVelocity();
    }

    private void setVoltage(double voltage) {
        REVOLVER_MOTOR.setOutput(MotorProperties.ControlMode.VOLTAGE, voltage);
    }
}