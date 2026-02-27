package frc.robot.utilities;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Translation2d;

import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.utilities.FieldConstants.*;
import static java.lang.Math.abs;

public class ZoneUtilities {
    private static final double LOOKAHEAD_TRENCH_TIME = 0.5; //todo tune
    private static final Debouncer TRENCH_DEBOUNCER = new Debouncer(0.1, Debouncer.DebounceType.kBoth);

    public static boolean willBeInTrenchZone() {
        final Translation2d currentPose = POSE_ESTIMATOR.getPose().getTranslation();
        final Translation2d futurePose = POSE_ESTIMATOR.predictFuturePose(LOOKAHEAD_TRENCH_TIME).getTranslation();
        final Translation2d midPose = currentPose.plus(futurePose).times(0.5);

        return TRENCH_DEBOUNCER.calculate(isInTrench(currentPose) || isInTrench(futurePose) || isInTrench(midPose));
    }

    private static boolean isInTrench(Translation2d pose) {
        final double foldedX = HALF_FIELD_LENGTH - abs(pose.getX() - HALF_FIELD_LENGTH);
        final double foldedY = HALF_FIELD_WIDTH - abs(pose.getY() - HALF_FIELD_WIDTH);

        return BOTTOM_TRENCH.contains(new Translation2d(foldedX, foldedY));
    }
}
