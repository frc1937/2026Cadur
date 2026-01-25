package frc.robot.subsystems.intake;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;

import static frc.lib.math.Conversions.mpsToRps;
import static frc.robot.RobotContainer.SWERVE;
import static frc.robot.subsystems.intake.IntakeConstants.*;

public class Intake extends GenericSubsystem {
    /**
     * Intake at the speed of max(robot velocity * 2, 3mps) to ensure optimal ball handling.
     * Never stops
     * @return a command that continuously adjusts intake velocity
     */
    public Command enableIntakeAdjusted() {
        return new FunctionalCommand(
                () -> {},
                () -> {
                    final double targetTangentialVelocity = Math.max(
                            2 * SWERVE.getRobotRelativeVelocity().vxMetersPerSecond,
                            MINIMUM_INTAKE_SPEED_TANGENTIAL_MPS
                    );

                    INTAKE_MOTOR.setOutput(MotorProperties.ControlMode.VELOCITY, mpsToRps(targetTangentialVelocity, INTAKE_WHEEL_DIAMETER_METERS));
                },
                (interrupt) -> {},
                () -> false,
                this
        );
    }

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