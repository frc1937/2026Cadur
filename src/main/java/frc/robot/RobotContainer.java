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
import frc.robot.utilities.ZoneUtilities;

import static frc.robot.poseestimation.PoseEstimatorConstants.TURRET_CAMERA;

public class RobotContainer {
    public static final BuiltInAccelerometer ACCELEROMETER = new BuiltInAccelerometer();

    public static final PoseEstimator POSE_ESTIMATOR = new PoseEstimator(
            new Camera[]{TURRET_CAMERA},
            null
    );

    public static final Swerve SWERVE = new Swerve();
    public static final Turret TURRET = new Turret();

    public static final Trigger IS_IN_TRENCH = new Trigger(ZoneUtilities::willBeInTrench);
    public static final Trigger IS_IN_TRENCH_AREA = new Trigger(ZoneUtilities::willBeInTrenchArea);
    public static final Trigger IS_HUB_ACTIVE = new Trigger(() -> MatchStateTracker.getInstance().isHubActive());
    
    public static final Hood HOOD = new Hood();
    public static final Flywheel FLYWHEEL = new Flywheel();
    public static final Intake INTAKE = new Intake();
    public static final Kicker KICKER = new Kicker();
    public static final Revolver REVOLVER = new Revolver();
    public static final Leds LEDS = new Leds();

    public static final ShooterStates SHOOTER_STATES = new ShooterStates();
    public static final ShootingCalculator SHOOTING_CALCULATOR = new ShootingCalculator();
    public static final Questionnaire QUESTIONNAIRE = new Questionnaire();

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
        HOOD.printPose();
    }
}