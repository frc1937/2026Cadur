package frc.robot;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.flippable.Flippable;
import frc.robot.commands.Questionnaire;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.poseestimation.PoseEstimator;
import frc.robot.poseestimation.camera.Camera;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.turret.Turret;

import static frc.robot.utilities.PathPlannerConstants.*;

public class RobotContainer {
    public static final BuiltInAccelerometer ACCELEROMETER = new BuiltInAccelerometer();

    public static final PoseEstimator POSE_ESTIMATOR = new PoseEstimator(
            new Camera[]{},
            null
    );

    public static final Turret TURRET = new Turret();
    public static final Flywheel FLYWHEEL = new Flywheel();
    public static final Arm ARM = new Arm();
    public static final Swerve SWERVE = new Swerve();
    public static final Leds LEDS = new Leds();
    public static final Questionnaire QUESTIONNAIRE = new Questionnaire();

    public static final FollowPath.Builder PATH_BUILDER = new FollowPath.Builder(
            SWERVE,
            POSE_ESTIMATOR::getCurrentPose,
            SWERVE::getFieldRelativeVelocity,
            speeds -> SWERVE.driveRobotRelative(speeds, true),
            BLINE_TRANSLATION_PID,
            BLINE_ROTATION_PID,
            BLINE_CROSS_TRACK_PID
    );

    public RobotContainer() {
        DriverStation.silenceJoystickConnectionWarning(true);

        Flippable.init();
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
        ARM.printPose();
    }
}