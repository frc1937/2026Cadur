package frc.robot.subsystems.shooter.kicker;


import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.characterization.FindMaxSpeedCommand;
import frc.robot.subsystems.shooter.ShooterStates;

import static frc.lib.generic.hardware.motor.MotorProperties.ControlMode.VELOCITY;
import static frc.lib.generic.hardware.motor.MotorProperties.ControlMode.VOLTAGE;
import static frc.robot.RobotContainer.*;
import static frc.robot.subsystems.shooter.ShooterStates.ShooterState.SHOOTING_HUB;
import static frc.robot.subsystems.shooter.ShooterStates.ShooterState.SHOOTING_HUB_KICKER_ACCELERATING;
import static frc.robot.subsystems.shooter.kicker.KickerConstants.KICKER_MOTOR;
import static java.lang.Math.abs;

public class Kicker extends GenericSubsystem {
    private final Debouncer accelerationDebouncer = new Debouncer(0.08, Debouncer.DebounceType.kBoth);

    private final static double FLYWHEEL_MPS_TO_KICKER_MPS = 1.75;
    private final double KICKER_VOLTAGE = 5;

    public Command followState() {
        return run(() -> {
            switch (SHOOTER_STATES.getState()) {
                case IDLE, SHOOTING_PASSING_HUB_BLOCKED, NOTHING -> stopMotor();
                case SHOOTING_HUB, SHOOTING_HUB_KICKER_ACCELERATING -> handleHubShooting();
                case SHOOTING_PASSING -> {
                    if (TURRET.getTurretVelocity() < 0.5)
                        KICKER_MOTOR.setOutput(VOLTAGE, KICKER_VOLTAGE);
                    else
                        KICKER_MOTOR.stopMotor();
                }
            }
        });
    }

    public Command findMaxVelocity() {
        return new FindMaxSpeedCommand(KICKER_MOTOR, this);
    }

    public Command cruiseAtMaxVelocity() {
        return runEnd(
                () -> KICKER_MOTOR.setOutput(VELOCITY, 85),
                KICKER_MOTOR::stopMotor
        );
    }

    public Command run() {
        return runEnd(
                () -> KICKER_MOTOR.setOutput(VOLTAGE, KICKER_VOLTAGE),
                KICKER_MOTOR::stopMotor
        );
    }

    private double getAdjustedFlywheelSurfaceSpeed() {
        return FLYWHEEL_MPS_TO_KICKER_MPS * SHOOTING_CALCULATOR.getResults().flywheelRPS();
    }

    private void stopMotor() {
        KICKER_MOTOR.stopMotor();
    }

    private void handleHubShooting() {
        if (!SHOOTER_STATES.isReadyToShoot()) {
            stopMotor();
            return;
        }

        final double targetRPS = 85;// Math.min(getAdjustedFlywheelSurfaceSpeed(), 81);
        KICKER_MOTOR.setOutput(VELOCITY, targetRPS);

        final boolean isAccelerating = accelerationDebouncer.calculate(abs(KICKER_MOTOR.getSystemVelocity() - targetRPS) > 20);

        ShooterStates.ShooterState targetState = isAccelerating
                ? SHOOTING_HUB_KICKER_ACCELERATING
                : SHOOTING_HUB;
        CommandScheduler.getInstance().schedule(SHOOTER_STATES.setState(targetState));
    }
}