package frc.robot.subsystems.shooter.flywheels;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.characterization.FindMaxSpeedCommand;
import frc.lib.generic.hardware.motor.MotorProperties;
import frc.robot.subsystems.shooter.ShooterStates;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.*;
import static frc.lib.generic.hardware.motor.MotorProperties.ControlMode.*;
import static frc.robot.RobotContainer.SHOOTING_CALCULATOR;
import static frc.robot.subsystems.shooter.flywheels.FlywheelConstants.*;
import static java.lang.Math.abs;

public class Flywheel extends GenericSubsystem {
    private final Debouncer currentDebouncer = new Debouncer(0.025, Debouncer.DebounceType.kFalling);

    public Command followState(ShooterStates states) {
        return run(() -> {
            switch (states.getState()) {
                case IDLE -> MASTER_FLYWHEEL_MOTOR.stopMotor();
                case SHOOTING_HUB -> setTargetSpeed(SHOOTING_CALCULATOR.getResults().flywheelRPS());
                case SHOOTING_PASSING -> setTargetSpeed(20); //todo tune, more sophisticated.
            }
        });
    }

    public boolean isReadyToShootPhysics() {
        return abs(SHOOTING_CALCULATOR.getResults().flywheelRPS() -
                MASTER_FLYWHEEL_MOTOR.getSystemVelocity()) < FLYWHEEL_SHOOTING_SPEED_TOLERANCE_RPS;
    }

    public Command getMaxValues() {
        return new FindMaxSpeedCommand(MASTER_FLYWHEEL_MOTOR, this);
    }

    public Command setTarget(double RPS) {
        return new FunctionalCommand(
                () -> {},
                () -> setTargetSpeed(RPS),
                (interrupted) -> MASTER_FLYWHEEL_MOTOR.stopMotor(),
                () -> false,
                this
        );
    }

    public Command setTarget(DoubleSupplier RPS) {
        return new FunctionalCommand(
                () -> {},
                () -> setTargetSpeed(RPS.getAsDouble()),
                (interrupted) -> MASTER_FLYWHEEL_MOTOR.stopMotor(),
                () -> false,
                this
        );
    }

    public Command stop() {
        return Commands.runOnce(MASTER_FLYWHEEL_MOTOR::stopMotor, this);
    }

    public boolean isAtGoal() {
        return MASTER_FLYWHEEL_MOTOR.isAtVelocitySetpoint();
    }

    public double getFlywheelVelocity() {
        return MASTER_FLYWHEEL_MOTOR.getSystemVelocity();
    }

    public double getFlywheelTargetVelocity() {
        return MASTER_FLYWHEEL_MOTOR.getClosedLoopTarget();
    }

    @Override
    public void periodic() {
        if (FLYWHEEL_MECHANISM == null) return;

        FLYWHEEL_MECHANISM.updateCurrentSpeed(getFlywheelVelocity());
        FLYWHEEL_MECHANISM.updateTargetSpeed(getFlywheelTargetVelocity());
    }

    @Override
    public SysIdRoutine.Config getSysIdConfig() {
        return SYSID_FLYWHEEL_CONFIG;
    }

    @Override
    public void sysIdDrive(double voltage) {
        MASTER_FLYWHEEL_MOTOR.setOutput(VOLTAGE, voltage);
    }

    @Override
    public void sysIdUpdateLog(SysIdRoutineLog log) {
        log.motor("FLYWHEEL_MASTER_VELOCITY" + MASTER_FLYWHEEL_MOTOR.getDeviceID())
                .voltage(Volts.of(MASTER_FLYWHEEL_MOTOR.getVoltage()))
                .angularPosition(Rotations.of(MASTER_FLYWHEEL_MOTOR.getSystemPosition()))
                .angularVelocity(RotationsPerSecond.of(MASTER_FLYWHEEL_MOTOR.getSystemVelocity()));
    }

    private void setTargetSpeed(double targetVelocityRPS) {
        final boolean inTolerance = abs(MASTER_FLYWHEEL_MOTOR.getSystemVelocity() - targetVelocityRPS) <= FLYWHEEL_SHOOTING_SPEED_TOLERANCE_RPS;
        final boolean currentControl = currentDebouncer.calculate(inTolerance);

        final MotorProperties.ControlMode mode = currentControl ? BANG_BANG_CURRENT : BANG_BANG_DUTY_CYCLE;

        MASTER_FLYWHEEL_MOTOR.setOutput(mode, targetVelocityRPS);
    }
}