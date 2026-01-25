package frc.robot.utilities;

import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.util.flippable.Flippable;
import org.littletonrobotics.junction.AutoLogOutput;

public class MatchStateTracker {//TODO add more functionality like shooting mode
    @AutoLogOutput(key = "IsHubActive")
    public static boolean isHubActive() {
        final String msg = DriverStation.getGameSpecificMessage();
        if (!DriverStation.isTeleop() || (!"R".equals(msg) && !"B".equals(msg))) return true;

        return Flippable.isRedAlliance() == isRedHubActive("R".equals(msg));
    }

    private static boolean isRedHubActive(boolean didRedWinAuto) {
        final int timeFrame = getCurrentTimeFrame();

        if (timeFrame == -1) return true;

        return didRedWinAuto == (timeFrame % 2 == 0);
    }

    private static int getCurrentTimeFrame() {
        final double matchTime = DriverStation.getMatchTime();

        if (matchTime <= 30 || matchTime > 130) return -1;

        double matchTimeLeft = 130 - matchTime;
        return (int) (matchTimeLeft / 25) + 1;
    }
}