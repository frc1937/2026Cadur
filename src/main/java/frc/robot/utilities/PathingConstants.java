package frc.robot.utilities;

import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.util.LocalADStarAK;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.swerve.SwerveConstants;
import org.littletonrobotics.junction.Logger;

import static frc.lib.generic.visualization.DrawUtils.TWO_PI;
import static frc.robot.GlobalConstants.*;
import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.RobotContainer.SWERVE;
import static frc.robot.subsystems.swerve.SwerveConstants.*;
import static frc.robot.subsystems.swerve.SwerveModuleConstants.*;

public class PathingConstants {
    public static final RobotConfig ROBOT_CONFIG = getRobotConfig();
    public static final double HANDOFF_RADIUS = 0.25;

    public static final PathConstraints PATH_PLANNER_CONSTRAINTS = IS_SIMULATION
            ? new PathConstraints(SwerveConstants.MAX_SPEED_MPS, 2, 6, 4)
            : new PathConstraints(SwerveConstants.MAX_SPEED_MPS, MAX_ACCELERATION_MPSSq, MAX_OMEGA_VELOCITY_DEG_PER_S * Math.PI/180.0,
            MAX_OMEGA_ACCELERATION_DEG_PER_SSQ * Math.PI / 180.0);
    //todo tune above

    public static final PIDController
            BLINE_TRANSLATION_PID = new PIDController(4.05, 0, 0.5),
            BLINE_ROTATION_PID = new PIDController(4.5, 0, 0.1),
            BLINE_CROSS_TRACK_PID = new PIDController(1.5, 0, 0);

    public static final FollowPath.Builder PATH_BUILDER = new FollowPath.Builder(
            SWERVE,
            POSE_ESTIMATOR::getPose,
            SWERVE::getRobotRelativeVelocity,
            speeds -> SWERVE.driveRobotRelative(speeds, true),
            BLINE_TRANSLATION_PID,
            BLINE_ROTATION_PID,
            BLINE_CROSS_TRACK_PID
    );

    public static void initializeBLine() {
        Pathfinding.setPathfinder(new LocalADStarAK());

        Path.setDefaultGlobalConstraints(new Path.DefaultGlobalConstraints(
                MAX_SPEED_MPS,
                MAX_ACCELERATION_MPSSq,
                MAX_OMEGA_VELOCITY_DEG_PER_S,
                MAX_OMEGA_ACCELERATION_DEG_PER_SSQ,
                0.1,
                2,
                HANDOFF_RADIUS
        ));

        FollowPath.setTranslationListLoggingConsumer((pair -> Logger.recordOutput(pair.getFirst(), pair.getSecond())));
        FollowPath.setBooleanLoggingConsumer((pair -> Logger.recordOutput(pair.getFirst(), pair.getSecond())));
        FollowPath.setDoubleLoggingConsumer((pair -> Logger.recordOutput(pair.getFirst(), pair.getSecond())));
        FollowPath.setPoseLoggingConsumer((pair -> Logger.recordOutput(pair.getFirst(), pair.getSecond())));

        CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
    }

    private static RobotConfig getRobotConfig() {
        ModuleConfig moduleConfig = new ModuleConfig(
                WHEEL_DIAMETER/2, MAX_SPEED_MPS, 1, DCMotor.getKrakenX60Foc(1),
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
