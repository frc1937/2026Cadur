package frc.robot.subsystems.swerve;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.PID;
import frc.lib.generic.ProfiledPID;
import frc.lib.generic.hardware.pigeon.Pigeon;
import frc.lib.generic.hardware.pigeon.PigeonConfiguration;
import frc.lib.generic.hardware.pigeon.PigeonFactory;
import frc.lib.generic.hardware.pigeon.PigeonSignal;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.GlobalConstants.IS_SIMULATION;
import static frc.robot.utilities.PathingConstants.ROBOT_CONFIG;
import static frc.robot.utilities.PortsConstants.SwervePorts.GYRO_PORT;

public class SwerveConstants {
    public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(ROBOT_CONFIG.moduleLocations);

    public static final double
            MAX_SPEED_MPS = 4.5,
            MAX_OMEGA_DEG_PER_S = 3 * 180;

    public static final double
            STEER_GEAR_RATIO = (150.0 / 7.0),
            DRIVE_GEAR_RATIO = (6.75),
            WHEEL_DIAMETER = 0.048923013788539564 * 2;

    protected static final SysIdRoutine.Config SYSID_DRIVE_CONFIG = new SysIdRoutine.Config(
            Volts.per(Second).of(1),
            Volts.of(2),
            Second.of(5)
    );

    protected static final double MAX_SKIDDING_RATIO = 1.5;

    public static final double
            DRIVE_NEUTRAL_DEADBAND = 0.10,
            ROTATION_NEUTRAL_DEADBAND = 0.10;

    protected static final PID PID_TRANSLATION_X_CONTROLLER = IS_SIMULATION
            ? new PID(1.2, 0, 0, 0.001)
            : new PID(1.105,0,0);

    protected static final PID PID_TRANSLATION_Y_CONTROLLER = IS_SIMULATION
            ? new PID(1.2, 0, 0, 0.001)
            : new PID(1.135,0.013,0);

    static final double YAW_ERROR_PID_KP = IS_SIMULATION ? 0.05 : 0.04;  //TODO: TUNE, speed per degree of yaw
    static final double PITCH_ERROR_PID_KP = IS_SIMULATION ? 0.05 : 0.04; //TODO: TUNE, speed per degree of pitch

    protected static final ProfiledPID SWERVE_ROTATION_CONTROLLER = IS_SIMULATION
            ? new ProfiledPID(0.2, 0, 0,0, new TrapezoidProfile.Constraints(360, 360))
            : new ProfiledPID(0.2205, 0, 0, new TrapezoidProfile.Constraints(360, 360));

    public static final Pigeon GYRO = PigeonFactory.createPigeon2("GYRO", GYRO_PORT);

    public static double yawOffset = 0;

    static {
        configureGyro();
        configureRotationController();
    }

    private static void configureGyro() {
        PigeonConfiguration configuration = new PigeonConfiguration();

        yawOffset = -89.64400482177734;

        configuration.mountPoseYawDegrees = yawOffset;
        configuration.mountPoseRollDegrees = -0.5925159454345703;
        configuration.mountPosePitchDegrees = 0.8338062763214111;

        GYRO.configurePigeon(configuration);

        GYRO.setupSignalUpdates(PigeonSignal.YAW, true);
        GYRO.setupSignalUpdates(PigeonSignal.PITCH, false);
        GYRO.setupSignalUpdates(PigeonSignal.ROLL, false);
    }

    private static void configureRotationController() {
        SWERVE_ROTATION_CONTROLLER.enableContinuousInput(-180, 180);
        SWERVE_ROTATION_CONTROLLER.setTolerance(1);

        PID_TRANSLATION_Y_CONTROLLER.setTolerance(0.03);
        PID_TRANSLATION_X_CONTROLLER.setTolerance(0.03);
    }
}
