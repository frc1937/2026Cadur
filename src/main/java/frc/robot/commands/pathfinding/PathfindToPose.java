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
import static frc.robot.utilities.PathingConstants.*;

public class PathfindToPose extends Command {
    private final Pose2d targetPose;
    private final double endVelocity;
    private Path resultPath;

    public PathfindToPose(Pose2d targetPose, double endVelocity) {
        this.targetPose = targetPose;
        this.endVelocity = endVelocity;

        addRequirements(SWERVE);
    }

    @Override
    public void initialize() {
        resultPath = null;

        Pathfinding.setStartPosition(POSE_ESTIMATOR.getPose().getTranslation());
        Pathfinding.setGoalPosition(targetPose.getTranslation());
    }

    @Override
    public boolean isFinished() {
        return Pathfinding.isNewPathAvailable();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) return;

        final PathPlannerPath foundPath = Pathfinding.getCurrentPath(
                PATH_PLANNER_CONSTRAINTS,
                new GoalEndState(endVelocity, targetPose.getRotation()));

        if (foundPath != null)
            this.resultPath = convertToBLine(foundPath);
    }

    public Path getGeneratedPath() {
        return resultPath;
    }

    private Path convertToBLine(PathPlannerPath foundPath) {
        final PathPlannerTrajectory trajectory =
                foundPath.generateTrajectory(SWERVE.getRobotRelativeVelocity(), POSE_ESTIMATOR.getCurrentAngle(), ROBOT_CONFIG);
        final List<PathPlannerTrajectoryState> states = trajectory.getStates();

        final Path.Waypoint[] elements = new Path.Waypoint[states.size()];

        for (int i = 0; i < elements.length; i++) {
            elements[i] = new Path.Waypoint(states.get(i).pose, HANDOFF_RADIUS, true);
        }

//        Pose2d[] posesForLogging = new Pose2d[elements.length];
//        for (int i = 0; i < elements.length; i++) {
//            posesForLogging[i] = new Pose2d(elements[i].translationTarget().translation(), elements[i].rotationTarget().rotation());
//        }
//        Logger.recordOutput("Pathfinding/ActualPath", posesForLogging);

        return new Path(elements);
    }
}