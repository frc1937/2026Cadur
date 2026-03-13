package frc.robot;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DriverStation;
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

import java.util.concurrent.atomic.AtomicInteger;

import static edu.wpi.first.wpilibj.RobotController.getBatteryVoltage;
import static frc.robot.poseestimation.PoseEstimatorConstants.TURRET_CAMERA;

public class RobotContainer {
    public static final BuiltInAccelerometer ACCELEROMETER = new BuiltInAccelerometer();

    public static final PoseEstimator POSE_ESTIMATOR = new PoseEstimator(
            new Camera[]{TURRET_CAMERA},
            null
    );

    public static final Swerve SWERVE = new Swerve();
    public static final Turret TURRET = new Turret();

    public static final Trigger IS_IN_TRENCH = new Trigger(ZoneUtilities::isInTrench);
    public static final Trigger IS_IN_TRENCH_AREA = new Trigger(ZoneUtilities::isInTrenchArea);
    public static final Trigger IS_IN_ALLIANCE_ZONE = new Trigger(ZoneUtilities::isInAllianceZone);

    public static final Hood HOOD = new Hood();
    public static final Flywheel FLYWHEEL = new Flywheel();
    public static final Intake INTAKE = new Intake();
    public static final Kicker KICKER = new Kicker();
    public static final Revolver REVOLVER = new Revolver();
    public static final Leds LEDS = new Leds();

    public static final ShooterStates SHOOTER_STATES = new ShooterStates();
    public static final MatchStateTracker MATCH_TRACKER = new MatchStateTracker();
    public static final ShootingCalculator SHOOTING_CALCULATOR = new ShootingCalculator();
    public static final Questionnaire QUESTIONNAIRE = new Questionnaire();

    public static final Trigger IS_HUB_ACTIVE = new Trigger(MATCH_TRACKER::isHubActive);

    public RobotContainer() {
        DriverStation.silenceJoystickConnectionWarning(true);

        Flippable.init();
        setupLEDsForBattery();
      
        ButtonControls.initializeButtons(ButtonControls.ButtonLayout.TUNE_HUB_SHOTS);
    }

    public Command getAutonomousCommand() {
        return QUESTIONNAIRE.getCommand();
    }

    public String getAutoName() {
        return QUESTIONNAIRE.getSelected();
    }

    private void setupLEDsForBattery() {
        final int BATTERY_LOW_THRESHOLD = 150;
        final AtomicInteger lowBatteryCounter = new AtomicInteger(0);

        final Trigger batteryLowTrigger = new Trigger(() -> {
            if (getBatteryVoltage() < 11.7) lowBatteryCounter.set(lowBatteryCounter.get() + 1);
            else lowBatteryCounter.set(0);

            return BATTERY_LOW_THRESHOLD < lowBatteryCounter.get();
        });

        batteryLowTrigger.onTrue(LEDS.showFor(Leds.LEDMode.BATTERY_LOW, 5));
    }

    public void updateComponentPoses() {
        TURRET.printPose();
        HOOD.printPose();
    }
}

/*
 * TODO
 *  trench automation (V)
 *  fix passing when in front of hub (test at green)
 *  IG Fix the sotm? didnt need fixing but oh well MAY NEED TO REVERT (V)
 *  leds showing the current state (grinbliz)
 *  when intake is going inside, move rollers V
 *  auto moved incrorectly after intaking V
 *  SHOOTING STATE for intake: half outside V
 *
 * What changed, to test at :grin: BLITZ
 * turret tolerance to 1.5degs
 * phase delay to 80ms
 * acceleration compensation in SOTM
 * changed some trench locations and hub locations
 * flywheel different isReadyToShoot.
 * changed LUT to warmup and to use newton convergence method
 *
 */