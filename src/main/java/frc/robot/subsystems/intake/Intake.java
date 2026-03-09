package frc.robot.subsystems.intake;


import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.GenericSubsystem;

import static edu.wpi.first.units.Units.*;
import static frc.lib.generic.hardware.motor.MotorProperties.ControlMode.*;
import static frc.lib.math.Conversions.mpsToRps;
import static frc.robot.RobotContainer.*;
import static frc.robot.subsystems.intake.IntakeConstants.*;
import static frc.robot.subsystems.intake.IntakeConstants.IntakeState.*;
import static java.lang.Math.abs;

public class Intake extends GenericSubsystem {
    private final Trigger isHardStop = new Trigger(() ->
            abs(INTAKE_EXTENSION_MOTOR.getSystemVelocity()) < 1 &&
                    abs(INTAKE_EXTENSION_MOTOR.getCurrent()) > 19)
            .debounce(0.1);

    private IntakeState state = RETRACTED;

    public Intake() {
        IS_IN_TRENCH.onTrue(setState(DEPLOYED));
    }

    public Command followState() {
        return run(() -> {
            INTAKE_ROLLER_MOTOR.setOutput(VOLTAGE, state.rollerVoltage);
            INTAKE_EXTENSION_MOTOR.setOutput(POSITION, state.position);
        });
    }

    public Command setState(IntakeState state) {
        return Commands.runOnce(() -> this.state = state);
    }

    public IntakeState getState() {
        return state;
    }

    public Command testRollerDeployment(double v) {
        return new FunctionalCommand(
                () -> {},
                () -> INTAKE_ROLLER_MOTOR.setOutput(VOLTAGE, v),
                (interrupt) -> INTAKE_ROLLER_MOTOR.stopMotor(),
                () -> false,
                this
        );
    }

    public Command grabBallsUnadjusted() {
        return new FunctionalCommand(
                () -> {},
                () -> INTAKE_ROLLER_MOTOR.setOutput(VOLTAGE, 4),
                (interrupt) -> INTAKE_ROLLER_MOTOR.stopMotor(),
                () -> false,
                this
        );
    }

    /**
     * Intake at the speed of max(robot velocity * 2, 3mps) to ensure optimal ball handling.
     * Never stops
     *
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

                    INTAKE_ROLLER_MOTOR.setOutput(VELOCITY, mpsToRps(targetTangentialVelocity, INTAKE_WHEEL_DIAMETER_METERS));
                },
                (interrupt) -> {},
                () -> false,
                this
        );
    }

    public Command calibrateIntakeZero() {
        return new FunctionalCommand(
                () -> INTAKE_EXTENSION_MOTOR.ignoreSoftwareLimits(true),
                () -> INTAKE_EXTENSION_MOTOR.setOutput(VOLTAGE, -1.2),
                (interrupt) -> {
                    INTAKE_EXTENSION_MOTOR.ignoreSoftwareLimits(false);
                    INTAKE_EXTENSION_MOTOR.stopMotor();

                    if (!interrupt)
                        INTAKE_EXTENSION_MOTOR.setMotorEncoderPosition(0);
                },
                isHardStop,
                this
        ).withTimeout(3);
    }

    public Command stopRoller() {
        return Commands.runOnce(INTAKE_ROLLER_MOTOR::stopMotor, this);
    }

    public double getSystemVelocity() {
        return INTAKE_ROLLER_MOTOR.getSystemVelocity();
    }

    @Override
    public SysIdRoutine.Config getSysIdConfig() {
        return SYSID_EXTENSION_CONFIG;
    }

    @Override
    public void sysIdDrive(double voltage) {
        INTAKE_EXTENSION_MOTOR.setOutput(VOLTAGE, voltage);
    }

    @Override
    public void sysIdUpdateLog(SysIdRoutineLog log) {
        log.motor("INTAKE_EXTENSION_MOTOR" + INTAKE_EXTENSION_MOTOR.getDeviceID())
                .voltage(Volts.of(INTAKE_EXTENSION_MOTOR.getVoltage()))
                .angularPosition(Rotations.of(INTAKE_EXTENSION_MOTOR.getSystemPosition()))
                .angularVelocity(RotationsPerSecond.of(INTAKE_EXTENSION_MOTOR.getSystemVelocity()));
    }

}