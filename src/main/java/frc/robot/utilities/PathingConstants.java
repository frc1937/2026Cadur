package frc.robot.utilities;

import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.swerve.SwerveConstants;
import org.littletonrobotics.junction.Logger;

import static frc.robot.GlobalConstants.*;
import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.RobotContainer.SWERVE;
import static frc.robot.subsystems.swerve.SwerveConstants.*;
import static frc.robot.subsystems.swerve.SwerveModuleConstants.*;

public class PathingConstants {
    public static final RobotConfig ROBOT_CONFIG = getRobotConfig();
    public static final int SAMPLED_POSE_INDICES = 8;

    public static final PathConstraints PATH_PLANNER_CONSTRAINTS =
            IS_SIMULATION
            ? new PathConstraints(SwerveConstants.MAX_SPEED_MPS, 2, 6, 4)
            : new PathConstraints(SwerveConstants.MAX_SPEED_MPS, 3.3, Math.PI * 1.3, Math.PI * 1.3);

    public static final PIDController
            BLINE_TRANSLATION_PID = new PIDController(4, 0, 0),
            BLINE_ROTATION_PID = new PIDController(8, 0, 0),
            BLINE_CROSS_TRACK_PID = new PIDController(2, 0, 0);

    public static final FollowPath.Builder PATH_BUILDER = new FollowPath.Builder(
            SWERVE,
            POSE_ESTIMATOR::getCurrentPose,
            SWERVE::getRobotRelativeVelocity,
            speeds -> SWERVE.driveRobotRelative(speeds, true),
            BLINE_TRANSLATION_PID,
            BLINE_ROTATION_PID,
            BLINE_CROSS_TRACK_PID
    );

    public static void initializeBLine() {
        Path.setDefaultGlobalConstraints(new Path.DefaultGlobalConstraints(
                SwerveConstants.MAX_SPEED_MPS,
                12,
                MAX_OMEGA_DEG_PER_S,
                860,
                0.03,
                1,
                0.2
        ));

        FollowPath.setTranslationListLoggingConsumer((pair -> Logger.recordOutput(pair.getFirst(), pair.getSecond())));
        FollowPath.setBooleanLoggingConsumer((pair -> Logger.recordOutput(pair.getFirst(), pair.getSecond())));
        FollowPath.setDoubleLoggingConsumer((pair -> Logger.recordOutput(pair.getFirst(), pair.getSecond())));
        FollowPath.setPoseLoggingConsumer((pair -> Logger.recordOutput(pair.getFirst(), pair.getSecond())));

        CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
    }

    private static RobotConfig getRobotConfig() {
        ModuleConfig moduleConfig = new ModuleConfig(
                WHEEL_DIAMETER/2, MAX_SPEED_MPS, 1, DCMotor.getKrakenX60(1),
                DRIVE_GEAR_RATIO, DRIVE_STATOR_CURRENT_LIMIT, 1
        );

        return new RobotConfig(ROBOT_MASS, ROBOT_MOI, moduleConfig,
                // Front Left (+X, +Y)
                new Translation2d(ROBOT_MODULE_LENGTH_X / 2.0, ROBOT_MODULE_LENGTH_Y / 2.0),
                // Front Right (+X, -Y)
                new Translation2d(ROBOT_MODULE_LENGTH_X / 2.0, -ROBOT_MODULE_LENGTH_Y / 2.0),
                // Back Left (-X, +Y)
                new Translation2d(-ROBOT_MODULE_LENGTH_X / 2.0, ROBOT_MODULE_LENGTH_Y / 2.0),
                // Back Right (-X, -Y)
                new Translation2d(-ROBOT_MODULE_LENGTH_X / 2.0, -ROBOT_MODULE_LENGTH_Y / 2.0));
    }
}
