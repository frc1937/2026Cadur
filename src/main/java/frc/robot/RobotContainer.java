package frc.robot;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.flippable.Flippable;
import frc.robot.commands.Questionnaire;
import frc.robot.poseestimation.PoseEstimator;
import frc.robot.poseestimation.camera.Camera;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.revolver.Revolver;
import frc.robot.subsystems.shooter.ShooterStates;
import frc.robot.subsystems.shooter.ShootingCalculator;
import frc.robot.subsystems.shooter.flywheels.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.kicker.Kicker;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utilities.MatchStateTracker;

import frc.robot.poseestimation.camera.CameraTransformCalibrator;

import static frc.robot.poseestimation.PoseEstimatorConstants.TURRET_CAMERA;


public class RobotContainer {
    public static final BuiltInAccelerometer ACCELEROMETER = new BuiltInAccelerometer();

    public static final PoseEstimator POSE_ESTIMATOR = new PoseEstimator(
            new Camera[]{TURRET_CAMERA},
            null
    );

    public static final Turret TURRET = new Turret();
    public static final Hood HOOD = new Hood();
    public static final Flywheel FLYWHEEL = new Flywheel();
    public static final Intake INTAKE = new Intake();
    public static final Kicker KICKER = new Kicker();
    public static final Revolver REVOLVER = new Revolver();
    public static final Swerve SWERVE = new Swerve();
    public static final Leds LEDS = new Leds();

    /**
     * Camera transform calibrator for the turret camera.
     *
     * Usage:
     *   1. Set CAMERA_PITCH_DEG to the pitch value read from the PhotonVision GUI
     *      (place a tag directly in front at the same height and read the displayed pitch).
     *   2. Set CAMERA_Z_METERS to the measured height of the camera lens above the turret centre.
     *   3. Deploy, disable the robot, drive/rotate near AprilTags.
     *   4. After ~30 observations the result appears on SmartDashboard under
     *      "CameraCalibration/JavaString". Paste it into TurretConstants.TURRET_CENTER_TO_CAMERA.
     */
    public static final CameraTransformCalibrator CAMERA_CALIBRATOR = new CameraTransformCalibrator(
            "TurretCamera",
            0,    // TODO: replace with camera pitch (degrees) from PhotonVision GUI
            0,    // TODO: replace with camera height above turret centre (metres), measured manually
            () -> TURRET.getSelfRelativePosition().getRadians()
    );

    public static final ShooterStates SHOOTER_STATES = new ShooterStates();
    public static final ShootingCalculator SHOOTING_CALCULATOR = new ShootingCalculator();
    public static final Questionnaire QUESTIONNAIRE = new Questionnaire();

    public RobotContainer() {
        DriverStation.silenceJoystickConnectionWarning(true);

        Flippable.init();
        MatchStateTracker.init();
        setupLEDsForBattery();

        ButtonControls.initializeButtons(ButtonControls.ButtonLayout.TELEOP);
    }

    public Command getAutonomousCommand() {
        return QUESTIONNAIRE.getCommand();
    }

    public String getAutoName() {
        return QUESTIONNAIRE.getSelected();
    }

    private void setupLEDsForBattery() {
        final int LOW_BATTERY_THRESHOLD = 150;
        final int[] lowBatteryCounter = {0};

        final Trigger batteryLowTrigger = new Trigger(() -> {
            if (RobotController.getBatteryVoltage() < 11.7)
                lowBatteryCounter[0]++;

            return LOW_BATTERY_THRESHOLD < lowBatteryCounter[0];
        });

        batteryLowTrigger.onTrue(LEDS.setLEDStatus(Leds.LEDMode.BATTERY_LOW, 5));
    }

    public void updateComponentPoses() {
        TURRET.printPose();
        HOOD.printPose();
    }
}