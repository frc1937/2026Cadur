package frc.robot.commands.pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.lib.BLine.Path;

import java.util.Set;

import static frc.robot.RobotContainer.SWERVE;
import static frc.robot.utilities.PathingConstants.PATH_BUILDER;

public class PathfindingCommands {
    public static Command pathfindAndFollow(Pose2d targetPose) {
        final PathfindToPose pathfinder = new PathfindToPose(targetPose);

        return pathfinder.andThen(new DeferredCommand(() -> {
            final Path generatedPath = pathfinder.getGeneratedPath();

            if (generatedPath == null) return Commands.none();

            return PATH_BUILDER.build(generatedPath);
        }, Set.of(SWERVE)));
    }
}
