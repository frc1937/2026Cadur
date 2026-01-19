package frc.robot.commands.pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;

import java.util.Set;

import static frc.robot.RobotContainer.SWERVE;

public class PathfindingCommands {
    public static Command pathfindToPose(Pose2d targetPose) {
        return new DeferredCommand(
                () -> new GeneratePath(targetPose, 0),
                Set.of(SWERVE)
        );
    }
}