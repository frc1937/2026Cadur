package frc.robot.utilities;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.util.flippable.Flippable;
import org.littletonrobotics.junction.AutoLogOutput;

import java.util.Optional;

import static frc.robot.RobotContainer.SHOOTING_CALCULATOR;

public class MatchStateTracker {
    private static final Timer TELEOP_TIMER = new Timer();

    private static final double[] BASE_TIMES = {0.0, 10.0, 35.0, 60.0, 85.0, 110.0, 140.0};

    private static final boolean[] WINNER_SCHEDULE = {true, true, false, true, false, true};
    private static final boolean[] LOSER_SCHEDULE  = {true, false, true, false, true, true};

    private static final double START_OFFSET = -(SHOOTING_CALCULATOR.getMinTimeOfFlight() + 1.0);
    private static final double END_OFFSET   = 3.0 - (SHOOTING_CALCULATOR.getMaxTimeOfFlight() + 2.0);

    public enum Shift {
        TRANSITION, SHIFT1, SHIFT2, SHIFT3, SHIFT4, ENDGAME, AUTO, DISABLED;

        public static Shift fromIndex(int i) {
            return (i >= 0 && i < 6) ? values()[i] : ENDGAME;
        }
    }

    public record ShiftInfo(Shift shift, boolean hubActive, double elapsed, double remaining) {}

    private static Optional<Boolean> manualOverride = Optional.empty();

    private static boolean fmsRedWonAuto = false;
    private static boolean gameDataReceived = false;
    private static boolean ignoreHubState = false;

    public static void initialize() {
        TELEOP_TIMER.restart();

        final String msg = DriverStation.getGameSpecificMessage();
        gameDataReceived = !msg.isEmpty();

        if (gameDataReceived) {
            fmsRedWonAuto = msg.charAt(0) == 'R';
        } else {
            DriverStation.reportWarning("[MatchStateTracker] No FMS data! Co-pilot must override.", false);
        }
    }

    public static boolean isHubActive() {
        return ignoreHubState || getCompensatedShiftInfo().hubActive();
    }

    @AutoLogOutput(key = "Dashboard/DidReceiveGameData")
    public static boolean isGameDataReceived() { return gameDataReceived; }

    @AutoLogOutput(key = "Dashboard/ShiftTime")
    public static ShiftInfo getOfficialShiftInfo() { return getShiftInfo(false); }

    @AutoLogOutput(key = "Dashboard/CompensatedShiftTime")
    public static ShiftInfo getCompensatedShiftInfo()  { return getShiftInfo(true); }

    private static ShiftInfo getShiftInfo(boolean applyFudge) {
        if (DriverStation.isAutonomousEnabled())
            return new ShiftInfo(Shift.AUTO, true, 0, 20.0);

        if (!DriverStation.isEnabled())
            return new ShiftInfo(Shift.DISABLED, false, 0, 0);

        boolean[] schedule = didOurAllianceWin() ? WINNER_SCHEDULE : LOSER_SCHEDULE;
        double now = TELEOP_TIMER.get();

        double[] boundaryTimes = new double[7];
        for (int i = 0; i <= 6; i++) boundaryTimes[i] = getAdjustedTime(i, schedule, applyFudge);

        int currentShiftIndex = 5;
        for (int i = 0; i < 6; i++) {
            if (now >= boundaryTimes[i] && now < boundaryTimes[i + 1]) {
                currentShiftIndex = i;
                break;
            }
        }

        int startIndex = currentShiftIndex;
        while (startIndex > 0 && schedule[startIndex] == schedule[startIndex - 1]) startIndex--;

        int endIndex = currentShiftIndex;
        while (endIndex < 5 && schedule[endIndex] == schedule[endIndex + 1]) endIndex++;

        return new ShiftInfo(
                Shift.fromIndex(currentShiftIndex),
                schedule[currentShiftIndex],
                now - boundaryTimes[startIndex],
                boundaryTimes[endIndex + 1] - now
        );
    }

    private static double getAdjustedTime(int index, boolean[] schedule, boolean applyFudge) {
        final double time = BASE_TIMES[index];

        if (!applyFudge || index == 0 || index == 6) return time;

        if (schedule[index - 1] && !schedule[index])  return time + END_OFFSET;
        if (!schedule[index - 1] && schedule[index])  return time + START_OFFSET;

        return time;
    }

    private static boolean didOurAllianceWin() {
        return manualOverride.orElse(fmsRedWonAuto) == Flippable.isRedAlliance();
    }

    //Did red win
    public static void setManualOverride(boolean redWon) { manualOverride = Optional.of(redWon); }
    public static void setIgnoreHubState(boolean ignore) { ignoreHubState = ignore; }
}