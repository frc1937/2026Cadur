package frc.robot.commands.pathfinding;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.BLine.Path;

import java.util.List;

import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.RobotContainer.SWERVE;
import static frc.robot.utilities.PathingConstants.PATHPLANNER_CONSTRAINTS;
import static frc.robot.utilities.PathingConstants.ROBOT_CONFIG;

public class PathfindToPose extends Command {
    private final Pose2d targetPose;
    private Path resultPath;

    public PathfindToPose(Pose2d targetPose) {
        this.targetPose = targetPose;
        addRequirements(SWERVE);
    }

    @Override
    public void initialize() {
        resultPath = null;
        Pathfinding.setStartPosition(POSE_ESTIMATOR.getCurrentPose().getTranslation());
        Pathfinding.setGoalPosition(targetPose.getTranslation());
    }

    @Override
    public boolean isFinished() {
        return Pathfinding.isNewPathAvailable();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) return;

        final PathPlannerPath foundPath = Pathfinding.getCurrentPath(PATHPLANNER_CONSTRAINTS, new GoalEndState(0, targetPose.getRotation()));
        if (foundPath != null) {
            this.resultPath = convertToBLine(foundPath);
        }
    }

    public Path getGeneratedPath() {
        return resultPath;
    }

    private Path convertToBLine(PathPlannerPath foundPath) {
        final PathPlannerTrajectory trajectory = foundPath.generateTrajectory(SWERVE.getRobotRelativeVelocity(), POSE_ESTIMATOR.getCurrentAngle(), ROBOT_CONFIG);
        final List<PathPlannerTrajectoryState> states = trajectory.getStates();
        final int stride = 8;

        final Path.Waypoint[] elements = new Path.Waypoint[states.size() / stride + 1];
        for (int i = 0; i < elements.length - 1; i++) {
            elements[i] = new Path.Waypoint(states.get(i * stride).pose, 0.25, true);
        }
        elements[elements.length - 1] = new Path.Waypoint(targetPose, 0.25, true);
        return new Path(elements);
    }
}