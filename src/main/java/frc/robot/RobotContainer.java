package frc.robot;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.flippable.Flippable;
import frc.lib.util.objectdetection.DetectionCamera;
import frc.lib.util.objectdetection.DetectionCameraFactory;
import frc.robot.commands.Questionnaire;
import frc.robot.poseestimation.PoseEstimator;
import frc.robot.poseestimation.camera.Camera;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.revolver.Revolver;
import frc.robot.subsystems.shooter.flywheels.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utilities.MatchStateTracker;


public class RobotContainer {
    public static final BuiltInAccelerometer ACCELEROMETER = new BuiltInAccelerometer();

    //TODO: Tune constatns below. FAHHH
    public static final DetectionCamera DETECTION_CAMERA = DetectionCameraFactory.createDetectionCamera("DetectionCamera", new Transform3d());

    public static final PoseEstimator POSE_ESTIMATOR = new PoseEstimator(
            new Camera[]{},
            null
    );

    public static final Turret TURRET = new Turret();
    public static final Flywheel FLYWHEEL = new Flywheel();
    public static final Hood HOOD = new Hood();
    public static final Intake INTAKE = new Intake();
    public static final Revolver REVOLVER = new Revolver();
    public static final Swerve SWERVE = new Swerve();
    public static final Leds LEDS = new Leds();
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