package frc.robot.commands.pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.lib.BLine.Path;

import java.util.Set;

import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.RobotContainer.SWERVE;
import static frc.robot.utilities.PathingConstants.PATH_BUILDER;

public class PathfindingCommands {
    public static Command pathfindAndFollow(Pose2d targetPose, Path.PathConstraints constraints, double endVelocity) {
        final PathfindToPose pathfinder = new PathfindToPose(targetPose, endVelocity);

        return pathfinder.andThen(new DeferredCommand(() -> {
            final Path generatedPath = pathfinder.getGeneratedPath();

            if (generatedPath == null)
                return Commands.none();

            if (constraints != null)
                generatedPath.setPathConstraints(constraints);

            return PATH_BUILDER.build(generatedPath);
        }, Set.of(SWERVE)));
    }

    public static Command pathfindAndFollow(Pose2d targetPose, double endVelocity) {
        return pathfindAndFollow(targetPose, null, endVelocity);
    }

    public static Command pathfindAndFollow(Pose2d targetPose, Path.PathConstraints constraints) {
        return pathfindAndFollow(targetPose, constraints, 0);
    }

    public static Command pathfindAndFollow(Pose2d targetPose) {
        return pathfindAndFollow(targetPose, null, 0);
    }

    /**
     * Pathfinds to the given translation while preserving the robot's heading at schedule time.
     * The angle is captured lazily (via DeferredCommand) to avoid stale rotation from
     * construction-time capture.
     */ //TODO end velocity doesn't work
    public static Command pathfindAndFollow(Translation2d targetLocation, Path.PathConstraints constraints, double endVelocity) {
        return new DeferredCommand(
                () -> pathfindAndFollow(new Pose2d(targetLocation, POSE_ESTIMATOR.getCurrentAngle()), constraints, endVelocity),
                Set.of(SWERVE)
        );
    }

    /** Pathfinds to the given translation while preserving the robot's heading at schedule time. */
    public static Command pathfindAndFollow(Translation2d targetLocation, Path.PathConstraints constraints) {
        return new DeferredCommand(
                () -> pathfindAndFollow(new Pose2d(targetLocation, POSE_ESTIMATOR.getCurrentAngle()), constraints, 0),
                Set.of(SWERVE)
        );
    }

    /** Pathfinds to the given translation while preserving the robot's heading at schedule time. */
    public static Command pathfindAndFollow(Translation2d targetLocation) {
        return new DeferredCommand(
                () -> pathfindAndFollow(new Pose2d(targetLocation, POSE_ESTIMATOR.getCurrentAngle()), null, 0),
                Set.of(SWERVE)
        );
    }
}