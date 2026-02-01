package frc.robot.utilities;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.flippable.Flippable;
import org.littletonrobotics.junction.AutoLogOutput;

public class MatchStateTracker {
    private static final Timer UPDATE_WINNER_TIMER = new Timer();
    private static boolean DID_RED_WIN_AUTO = notCashedRedWinAuto();

    public static void init() {
        UPDATE_WINNER_TIMER.start();

        new Trigger(() -> UPDATE_WINNER_TIMER.advanceIfElapsed(1))
                .onTrue(getUpdateWinnerCommand());
    }

    public static boolean didRedWinAuto() {
        return DID_RED_WIN_AUTO;
    }

    @AutoLogOutput(key = "IsHubActive")
    public static boolean isHubActive() {
        return Flippable.isRedAlliance() == isRedHubActive(didRedWinAuto());
    }

    private static boolean isRedHubActive(boolean didRedWinAuto) {
        final int timeFrame = getCurrentTimeFrame();

        if (timeFrame == -1) return true;

        return didRedWinAuto == (timeFrame % 2 == 0);
    }

    private static int getCurrentTimeFrame() {
        final double matchTime = DriverStation.getMatchTime();

        if (matchTime <= 30 || matchTime > 130) return -1;

        final double matchTimeLeft = 130 - matchTime;
        return (int) (matchTimeLeft / 25) + 1;
    }

    private static Command getUpdateWinnerCommand() {
        return new InstantCommand(() -> DID_RED_WIN_AUTO = notCashedRedWinAuto()).ignoringDisable(true);
    }

    private static boolean notCashedRedWinAuto() {
        final String msg = DriverStation.getGameSpecificMessage();
        return "R".equals(msg);
    }
}