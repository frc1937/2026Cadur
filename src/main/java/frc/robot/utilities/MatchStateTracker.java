package frc.robot.utilities;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.util.flippable.Flippable;
import org.littletonrobotics.junction.AutoLogOutput;

import java.util.Optional;

import static frc.robot.RobotContainer.SHOOTING_CALCULATOR;

public class MatchStateTracker {
    private final Timer TELEOP_TIMER = new Timer();

    private static final double[] BASE_TIMES = {0.0, 10.0, 35.0, 60.0, 85.0, 110.0, 140.0};

    private static final boolean[] WINNER_SCHEDULE = {true, true, false, true, false, true};
    private static final boolean[] LOSER_SCHEDULE  = {true, false, true, false, true, true};

    private double START_OFFSET;
    private double END_OFFSET;

    public enum Shift {
        TRANSITION, SHIFT1, SHIFT2, SHIFT3, SHIFT4, ENDGAME, AUTO, DISABLED;

        public static Shift fromIndex(int i) {
            return (i >= 0 && i < 6) ? values()[i] : ENDGAME;
        }
    }

    public record ShiftInfo(Shift shift, boolean hubActive, double elapsed, double remaining) {}

    private Optional<Boolean> manualOverride = Optional.empty();

    private boolean fmsRedWonAuto = false;
    private boolean gameDataReceived = false;
    private boolean ignoreHubState = false;

    public void initialize() {
        START_OFFSET = -(SHOOTING_CALCULATOR.getMinTimeOfFlight() + 1.0);
        END_OFFSET   = 3.0 - (SHOOTING_CALCULATOR.getMaxTimeOfFlight() + 2.0);

        TELEOP_TIMER.restart();

        final String msg = DriverStation.getGameSpecificMessage();
        gameDataReceived = !msg.isEmpty();

        if (gameDataReceived) {
            fmsRedWonAuto = msg.charAt(0) == 'R';
        } else {
            DriverStation.reportWarning("[MatchStateTracker] No FMS data! Co-pilot must override.", false);
        }
    }

    @AutoLogOutput(key = "MatchStateTracker/IsHubActive")
    public boolean isHubActive() {
        return ignoreHubState || getCompensatedShiftInfo().hubActive();
    }

    @AutoLogOutput(key = "MatchStateTracker/GameDataReceived")
    public boolean isGameDataReceived() {
        return gameDataReceived;
    }

    @AutoLogOutput(key = "MatchStateTracker/OfficialShiftTimeRemaining")
    public double getOfficialShiftTimeRemaining() {
        return getShiftInfo(false).remaining();
    }

    @AutoLogOutput(key = "MatchStateTracker/CompensatedShiftTimeRemaining")
    public double getCompensatedShiftTimeRemaining() {
        return getCompensatedShiftInfo().remaining();
    }

    public ShiftInfo getCompensatedShiftInfo() {
        return getShiftInfo(true);
    }

    public void setManualOverride(boolean redWon) {
        manualOverride = Optional.of(redWon);
    }

    public void toggleIgnoreHubState() {
        ignoreHubState = !ignoreHubState;
    }

    public boolean shouldIgnoreHubState() {
        return ignoreHubState;
    }

    private ShiftInfo getShiftInfo(boolean applyFudge) {
        if (DriverStation.isAutonomousEnabled())
            return new ShiftInfo(Shift.AUTO, true, 0.0, 20.0);

        if (!DriverStation.isEnabled())
            return new ShiftInfo(Shift.DISABLED, false, 0.0, 0.0);

        boolean[] schedule = didOurAllianceWin() ? WINNER_SCHEDULE : LOSER_SCHEDULE;
        double now = TELEOP_TIMER.get();

        double[] offsets = new double[7];
        if (applyFudge) {
            for (int i = 1; i <= 5; i++) {
                boolean prev = schedule[i - 1];
                boolean next = schedule[i];
                if (prev && !next)       offsets[i] = END_OFFSET;
                else if (!prev && next)  offsets[i] = START_OFFSET;
            }
        }

        int idx = schedule.length - 1;
        for (int i = 0; i < schedule.length; i++) {
            double start = BASE_TIMES[i]     + offsets[i];
            double end   = BASE_TIMES[i + 1] + offsets[i + 1];
            if (now >= start && now < end) {
                idx = i;
                break;
            }
        }

        double shiftStart = BASE_TIMES[idx]     + offsets[idx];
        double shiftEnd   = BASE_TIMES[idx + 1] + offsets[idx + 1];

        double elapsed   = now - shiftStart;
        double remaining = shiftEnd - now;

        if (idx > 0 && schedule[idx] == schedule[idx - 1]) {
            elapsed = now - (BASE_TIMES[idx - 1] + offsets[idx - 1]);
        }

        if (idx < schedule.length - 1 && schedule[idx] == schedule[idx + 1]) {
            remaining = (BASE_TIMES[idx + 2] + offsets[idx + 2]) - now;
        }

        return new ShiftInfo(Shift.fromIndex(idx), schedule[idx], elapsed, remaining);
    }

    private boolean didOurAllianceWin() {
        return manualOverride.orElse(fmsRedWonAuto) == Flippable.isRedAlliance();
    }
}