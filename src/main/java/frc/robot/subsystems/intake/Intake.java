package frc.robot.subsystems.intake;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;

import static frc.lib.generic.hardware.motor.MotorProperties.ControlMode.VOLTAGE;
import static frc.lib.math.Conversions.mpsToRps;
import static frc.robot.RobotContainer.SWERVE;
import static frc.robot.subsystems.intake.IntakeConstants.*;
import static java.lang.Math.abs;

public class Intake extends GenericSubsystem {
    private final Trigger isHardStop = new Trigger(() -> (abs(INTAKE_EXTENSION_MOTOR.getSystemVelocity()) < 1
            && abs(INTAKE_EXTENSION_MOTOR.getCurrent()) > 10)).debounce(0.1);

    public Command retractIntake() {
        return new FunctionalCommand(
                () -> {},
                () -> INTAKE_EXTENSION_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, INTAKE_RETRACTED_POSITION),
                (interrupt) -> INTAKE_EXTENSION_MOTOR.stopMotor(),
                () -> false,
                this
        );
    }

    public Command deployIntake() {
        return new FunctionalCommand(
                () -> {},
                () -> INTAKE_EXTENSION_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, INTAKE_DEPLOYED_POSITION),
                (interrupt) -> INTAKE_EXTENSION_MOTOR.stopMotor(),
                () -> false,
                this
        );
    }


    /**
     * Intake at the speed of max(robot velocity * 2, 3mps) to ensure optimal ball handling.
     * Never stops
     * @return a command that continuously adjusts intake velocity
     */
    public Command grabBallsAdjusted() {
        return new FunctionalCommand(
                () -> {},
                () -> {
                    final double targetTangentialVelocity = Math.max(
                            2 * SWERVE.getRobotRelativeVelocity().vxMetersPerSecond,
                            MINIMUM_INTAKE_SPEED_TANGENTIAL_MPS
                    );

                    INTAKE_GRAB_MOTOR.setOutput(MotorProperties.ControlMode.VELOCITY, mpsToRps(targetTangentialVelocity, INTAKE_WHEEL_DIAMETER_METERS));
                },
                (interrupt) -> {},
                () -> false,
                this
        );
    }


    /**
     * Recalibrates the hood zero point. This slowly drives the hood
     * down until we see a drop in velocity and a spike in stator current,
     * indicating that we've hit a hard stop.
     *
     * @return Command to run
     */
    public Command calibrateIntakeZero() { //todo test
        return new FunctionalCommand(
                () -> INTAKE_EXTENSION_MOTOR.ignoreSoftwareLimits(true),
                () -> INTAKE_EXTENSION_MOTOR.setOutput(VOLTAGE, -0.1),
                (interrupt) -> {
                    INTAKE_EXTENSION_MOTOR.ignoreSoftwareLimits(false);
                    INTAKE_EXTENSION_MOTOR.stopMotor();

                    if (!interrupt)
                        INTAKE_EXTENSION_MOTOR.setMotorEncoderPosition(0);
                },
                isHardStop,
                this
        ).withTimeout(2);
    }

    public Command stopGrabbing() {
        return Commands.runOnce(INTAKE_GRAB_MOTOR::stopMotor, this);
    }

    public double getSystemVelocity() {
        return INTAKE_GRAB_MOTOR.getSystemVelocity();
    }
}