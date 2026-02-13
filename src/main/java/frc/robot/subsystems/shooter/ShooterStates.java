package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.*;

import static frc.robot.RobotContainer.*;
import static java.lang.Math.abs;
import static java.lang.Math.hypot;

public class ShooterStates {
    public enum ShooterState {
        IDLE,
        SHOOTING_HUB,
        SHOOTING_PASSING,
    }

    private static ShooterState CURRENT_STATE = ShooterState.IDLE;

    public Command setCurrentState(ShooterState newState) {
        var command = switch (newState) {
            case IDLE ->
                    idleCommand();
            case SHOOTING_HUB ->
                    shootHubCommand();
            case SHOOTING_PASSING ->
                    passingCommand();
        };

        return Commands.runOnce(() -> CURRENT_STATE = newState).alongWith(command);
    }

    private Command shootHubCommand() {
        final ConditionalCommand shootBall = new ConditionalCommand(
                KICKER.releaseBall().alongWith(REVOLVER.enableRevolver()), //todo: figure out how to revolver
                KICKER.stop(),

                () -> {
                    final boolean isTurretReady = TURRET.isReadyToShoot();
                    final boolean isHoodReady = HOOD.isAtGoal();
                    final boolean isFlywheelReady = FLYWHEEL.isAtGoal();
                    final boolean isRobotStable =
                            hypot(SWERVE.getRobotRelativeVelocity().vxMetersPerSecond, SWERVE.getRobotRelativeVelocity().vyMetersPerSecond) <= 3.0
                         && abs(SWERVE.getRobotRelativeVelocity().omegaRadiansPerSecond) <= 0.5;

                    return isTurretReady && isHoodReady && isFlywheelReady && isRobotStable;
                }
        );

        return new ParallelCommandGroup(
                FLYWHEEL.trackHub(),
                HOOD.trackHub(),
                TURRET.trackHub(),

                new RepeatCommand(shootBall)
        );
    }

    private Command idleCommand() {
        return new ParallelCommandGroup(
                FLYWHEEL.stop(),
                KICKER.stop(),
                HOOD.stopHood(),
                TURRET.stopTurret() //TODO: Turret track HUB april TAG, set shooter and feeder to 0. Waiting on you ran.
        );
    }

    private Command passingCommand() {
        return new ParallelCommandGroup(
                FLYWHEEL.shootPassing(),
                REVOLVER.enableRevolver(),
                KICKER.releaseBall(),
                TURRET.trackPassingPoint()
        );
    }
}
