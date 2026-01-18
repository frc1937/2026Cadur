package frc.robot.utilities;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.swerve.SwerveConstants;
import org.json.simple.parser.ParseException;

import java.io.IOException;

import static frc.robot.GlobalConstants.IS_SIMULATION;

public class PathPlannerConstants {
    public static final RobotConfig ROBOT_CONFIG = getRobotConfig();

    public static final PathConstraints PATHPLANNER_CONSTRAINTS = IS_SIMULATION
            ? new PathConstraints(SwerveConstants.MAX_SPEED_MPS, 2, 6, 4)
            : new PathConstraints(SwerveConstants.MAX_SPEED_MPS, 3.3, Math.PI * 1.3, Math.PI * 1.3);

    public static final PIDController BLINE_TRANSLATION_PID = new PIDController(4, 0, 0);
    public static final PIDController BLINE_ROTATION_PID = new PIDController(5, 0, 0);
    public static final PIDController BLINE_CROSS_TRACK_PID = new PIDController(8, 0, 2);

    private static RobotConfig getRobotConfig() {
        try {
            return RobotConfig.fromGUISettings();
        } catch (IOException | ParseException e) {
            throw new RuntimeException(e);
        }
    }
}
