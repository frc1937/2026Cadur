package frc.robot.subsystems.shooter;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.VisualizeShot;

import static frc.robot.RobotContainer.*;
import static java.lang.Math.hypot;

public class ShooterStates {
    public enum ShooterState {
        IDLE,
        SHOOTING_HUB,
        SHOOTING_PASSING,
    }

    public Command setCurrentState(ShooterState newState) {
        return switch (newState) {
            case IDLE -> idleCommand();
            case SHOOTING_HUB -> shootHubCommand();
            case SHOOTING_PASSING -> passingCommand();
        };
    }

    private Command shootHubCommand() {
        final ConditionalCommand shootBall = new ConditionalCommand(
                KICKER.releaseBall().alongWith(REVOLVER.enableRevolver()).alongWith(new VisualizeShot()), //todo: figure out how to revolver
                KICKER.stop(),

                () -> {
                    final boolean isTurretReady = TURRET.isReadyToShoot();
                    final boolean isHoodReady = HOOD.isAtGoal();
                    final boolean isFlywheelReady = FLYWHEEL.isAtGoal();

                    final ChassisSpeeds robotVelocity = SWERVE.getRobotRelativeVelocity();

                    final boolean isRobotStable = hypot(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond) <= 5.0;

                    return isTurretReady && isHoodReady && isFlywheelReady && isRobotStable;
                }
        );

        return new ParallelCommandGroup(
                FLYWHEEL.trackHub(),
                HOOD.trackHub(),
                TURRET.trackHubForSOTM(),

                new RepeatCommand(shootBall)
        );
    }

    private Command idleCommand() {
        return new ParallelCommandGroup(
                FLYWHEEL.stop(),
                KICKER.stop(),
                HOOD.stopHood(),
                REVOLVER.stop(),
                TURRET.trackHubIdly()
        );
    }

    private Command passingCommand() {
        return new ParallelCommandGroup(
                FLYWHEEL.trackPassing(),
                TURRET.trackPassingPoint(),
                HOOD.stopHood(),
                REVOLVER.enableRevolver(),

                new RepeatCommand(KICKER.releaseBall())
        );
    }
}
