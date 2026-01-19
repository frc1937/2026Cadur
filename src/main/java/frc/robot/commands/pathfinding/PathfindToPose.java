package frc.robot.commands.pathfinding;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.lib.BLine.Path;

import java.util.List;

import static frc.robot.RobotContainer.*;
import static frc.robot.utilities.FieldConstants.FIELD_LENGTH;
import static frc.robot.utilities.FieldConstants.FIELD_WIDTH;
import static frc.robot.utilities.PathPlannerConstants.PATHPLANNER_CONSTRAINTS;
import static frc.robot.utilities.PathPlannerConstants.ROBOT_CONFIG;

public class PathfindToPose extends Command {
    private final Pose2d targetPose;

    public PathfindToPose(Pose2d targetPose) {
        this.targetPose = targetPose;
        addRequirements(SWERVE);
    }

    @Override
    public void initialize() {
        Pathfinding.ensureInitialized();
        Translation2d currentPose = POSE_ESTIMATOR.getCurrentPose().getTranslation();
        currentPose = new Translation2d(
                MathUtil.clamp(currentPose.getX(), 0, FIELD_LENGTH),
                MathUtil.clamp(currentPose.getY(), 0, FIELD_WIDTH)
        );

        Pathfinding.setStartPosition(currentPose);
        Pathfinding.setGoalPosition(targetPose.getTranslation());
    }

    @Override
    public boolean isFinished() {
        return Pathfinding.isNewPathAvailable();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted || !Pathfinding.isNewPathAvailable()) return;

        final PathPlannerPath foundPath = Pathfinding.getCurrentPath(PATHPLANNER_CONSTRAINTS, new GoalEndState(0, targetPose.getRotation()));
        if (foundPath == null) return;

        final int stride = 8;
        final List<PathPlannerTrajectoryState> states = foundPath.generateTrajectory(
                SWERVE.getRobotRelativeVelocity(),
                POSE_ESTIMATOR.getCurrentAngle(),
                ROBOT_CONFIG
        ).getStates();
        final Path.Waypoint[] elements = new Path.Waypoint[states.size() / stride + 1];

        for (int i = 0; i < elements.length - 1; i++) {
            elements[i] = new Path.Waypoint(states.get(i * stride).pose, 0.25, true);
        }
        elements[elements.length - 1] = new Path.Waypoint(targetPose, 0.25, true);

        final Path path = new Path(elements);
        if (!path.isValid()) return;
        CommandScheduler.getInstance().schedule(PATH_BUILDER.build(path));
    }
}