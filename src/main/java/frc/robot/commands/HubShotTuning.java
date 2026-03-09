package frc.robot.commands;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import static frc.robot.RobotContainer.*;
import static frc.robot.utilities.FieldConstants.HUB_TOP_POSITION;

public class HubShotTuning {
    private static final String KEY = "HubShotTuner/";
    private static final int SHOTS_TO_CONFIRM = 3;

    private final static LoggedNetworkNumber hoodAngleDegrees = new LoggedNetworkNumber(KEY + "hoodAngleDeg", 2.0);
    private final static LoggedNetworkNumber flywheelSpeedRPS = new LoggedNetworkNumber(KEY + "flywheelSpeedRPS", 4.0);

    private static int consecutiveMakes = 0;

    public static Command shootFromDashboard() {
        Trigger isAtGoal = new Trigger(() -> HOOD.isAtGoal() && FLYWHEEL.isAtGoal());

        return Commands.parallel(
                HOOD.setTarget(() -> hoodAngleDegrees.get() / 360.0),
                FLYWHEEL.setTarget(flywheelSpeedRPS::get),
                Commands.waitUntil(isAtGoal).andThen(KICKER.run().alongWith(REVOLVER.enableRevolver()))
        );
    }

    public static Command confirmMake() {
        return Commands.runOnce(() -> {
            consecutiveMakes++;
            Logger.recordOutput(KEY + "consecutiveMakes", consecutiveMakes);

            if (consecutiveMakes >= SHOTS_TO_CONFIRM) {
                saveToPreferences();
                consecutiveMakes = 0;
            }
        });
    }

    /**
     * Bind to operator button: "missed" — resets the streak
     */
    public static Command confirmMiss() {
        return Commands.runOnce(() -> {
            consecutiveMakes = 0;
            Logger.recordOutput(KEY + "consecutiveMakes", consecutiveMakes);
        });
    }

    private static void saveToPreferences() {
        final double dist = HUB_TOP_POSITION.get().toTranslation2d().getDistance(POSE_ESTIMATOR.getPose().getTranslation());
        final double hood = hoodAngleDegrees.get();
        final double flywheel = flywheelSpeedRPS.get();

        String prefix = String.format("Shot/%.3fm/", dist);
        Preferences.setDouble(prefix + "hoodDeg", hood);
        Preferences.setDouble(prefix + "flywheelRPS", flywheel);

        Logger.recordOutput(KEY + "lastSaved/distanceMeters", dist);
        Logger.recordOutput(KEY + "lastSaved/hoodDeg", hood);
        Logger.recordOutput(KEY + "lastSaved/flywheelRPS", flywheel);

        System.out.printf("[HubShotTuning] ✓ Saved: %.3fm → %.3f° hood, %.3f RPS%n", dist, hood, flywheel);
    }
}