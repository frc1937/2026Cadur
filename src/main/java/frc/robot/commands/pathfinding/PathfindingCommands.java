package frc.robot.commands.pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.lib.BLine.Path;

import java.util.Set;

import static frc.robot.RobotContainer.PATH_BUILDER;
import static frc.robot.RobotContainer.SWERVE;

public class PathfindingCommands {
    public static Command pathfindToPose(Pose2d targetPose) {
        return new DeferredCommand(
                () -> {
                    final GeneratePath generatePath = new GeneratePath(targetPose, 0);

                    return generatePath.andThen(() -> {
                        final Path path = generatePath.getPath();
                        if (!path.isValid())
                            return;
                        CommandScheduler.getInstance().schedule(PATH_BUILDER.build(path));
                    });
                },
                Set.of(SWERVE)
        );
    }
}