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
            MAX_SPEED_MPS = 4.238075246, //todo tune
            MAX_ACCELERATION_MPSSq = 12, //todo tune
            MAX_OMEGA_VELOCITY_DEG_PER_S = 3 * 180, //todo tune
            MAX_OMEGA_ACCELERATION_DEG_PER_SSQ = 850;   //todo tune

    public static final double
            STEER_GEAR_RATIO = (150.0 / 7.0),
            DRIVE_GEAR_RATIO = (6.75),
            WHEEL_DIAMETER = 0.0487205013788539564 * 2; //todo tune

    protected static final SysIdRoutine.Config SYSID_DRIVE_CONFIG = new SysIdRoutine.Config(
            Volts.per(Second).of(1.5),
            Volts.of(6),
            Second.of(5));

    public static final double
            DRIVE_NEUTRAL_DEADBAND = 0.10,
            ROTATION_NEUTRAL_DEADBAND = 0.10;

    protected static final PID PID_TRANSLATION_X_CONTROLLER = IS_SIMULATION
            ? new PID(1.2, 0, 0, 0.001)
            : new PID(1.105,0,0);

    protected static final PID PID_TRANSLATION_Y_CONTROLLER = IS_SIMULATION
            ? new PID(1.2, 0, 0, 0.001)
            : new PID(1.135,0.013,0);

    protected static final PID TRENCH_CORRECTION_Y_CONTROLLER = IS_SIMULATION
            ? new PID(20, 0, 0, 0.001)
            : new PID(5,0.0,0.05);

    protected static final PID SWERVE_ROTATION_PID = IS_SIMULATION
            ? new PID(10, 0, 0, 0.001)
            : new PID(0.8,0,0.002);

    protected static final ProfiledPID SWERVE_ROTATION_CONTROLLER = IS_SIMULATION
            ? new ProfiledPID(8, 0, 0,0, new TrapezoidProfile.Constraints(720, 720))
            : new ProfiledPID(0.2205, 0, 0, new TrapezoidProfile.Constraints(720, 720));

    public static final Pigeon GYRO = PigeonFactory.createPigeon2("GYRO", GYRO_PORT);

    static {
        configureGyro();
        configureRotationController();
    }

    private static void configureGyro() {
        PigeonConfiguration configuration = new PigeonConfiguration();

        configuration.mountPoseYawDegrees = 0.005188619252294302;
        configuration.mountPoseRollDegrees = 0.24638523161411285;
        configuration.mountPosePitchDegrees = -0.539567768573761;

        GYRO.configurePigeon(configuration);

        GYRO.setupSignalUpdates(PigeonSignal.YAW, true);
        GYRO.setupSignalUpdates(PigeonSignal.YAW_RATE, true);
        GYRO.setupSignalUpdates(PigeonSignal.PITCH, false);
        GYRO.setupSignalUpdates(PigeonSignal.ROLL, false);
    }

    private static void configureRotationController() {
        SWERVE_ROTATION_CONTROLLER.enableContinuousInput(-180, 180);
        SWERVE_ROTATION_CONTROLLER.setTolerance(1);

        SWERVE_ROTATION_PID.enableContinuousInput(-0.5, 0.5);
        SWERVE_ROTATION_PID.setTolerance(0.002);

        TRENCH_CORRECTION_Y_CONTROLLER.setTolerance(0.03);
        PID_TRANSLATION_Y_CONTROLLER.setTolerance(0.03);
        PID_TRANSLATION_X_CONTROLLER.setTolerance(0.03);
    }
}
