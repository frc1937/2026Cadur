package frc.robot.commands.pathfinding;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.BLine.Path;

import java.util.List;
import java.util.function.Supplier;

import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.RobotContainer.SWERVE;
import static frc.robot.utilities.FieldConstants.FIELD_LENGTH;
import static frc.robot.utilities.FieldConstants.FIELD_WIDTH;
import static frc.robot.utilities.PathPlannerConstants.PATHPLANNER_CONSTRAINTS;
import static frc.robot.utilities.PathPlannerConstants.ROBOT_CONFIG;

public class GeneratePath extends Command {
    private final Supplier<Pose2d> poseSupplier;
    private final Pose2d targetPose;
    private final PathConstraints constraints;
    private final GoalEndState goalEndState;

    private PathPlannerPath currentPath;

    public GeneratePath(Pose2d targetPose, double goalEndVelocity) {
        this.poseSupplier = POSE_ESTIMATOR::getCurrentPose;
        this.targetPose = targetPose;
        this.constraints = PATHPLANNER_CONSTRAINTS;
        this.goalEndState = new GoalEndState(goalEndVelocity, targetPose.getRotation());

        Pathfinding.ensureInitialized();
    }

    @Override
    public void initialize() {
        Translation2d currentPose = poseSupplier.get().getTranslation();
        currentPose = new Translation2d(
                MathUtil.clamp(currentPose.getX(), 0, FIELD_LENGTH),
                MathUtil.clamp(currentPose.getY(), 0, FIELD_WIDTH)
        );
        Pathfinding.setStartPosition(currentPose);
        Pathfinding.setGoalPosition(targetPose.getTranslation());
    }

    @Override
    public void execute() {
        if (Pathfinding.isNewPathAvailable()) {
            currentPath = Pathfinding.getCurrentPath(constraints, goalEndState);
        }
    }

    @Override
    public boolean isFinished() {
        return currentPath != null;
    }

    public Path getPath() {
        if (currentPath == null)
            return new Path();

        final int stride = 8;
        final List<PathPlannerTrajectoryState> states = currentPath.generateTrajectory(
                SWERVE.getFieldRelativeVelocity(),
                POSE_ESTIMATOR.getCurrentAngle(),
                ROBOT_CONFIG
        ).getStates();
        final Path.Waypoint[] elements = new Path.Waypoint[states.size() / stride + 1];

        for (int i = 0; i < elements.length - 1; i++) {
            elements[i] = new Path.Waypoint(states.get(i * stride).pose, 0.25, true);
        }

        elements[elements.length - 1] = new Path.Waypoint(targetPose, 0.25, true);
        return new Path(elements);
    }
}