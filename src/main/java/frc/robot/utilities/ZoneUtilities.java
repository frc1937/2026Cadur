package frc.robot.utilities;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.function.Predicate;

import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.utilities.FieldConstants.*;
import static java.lang.Math.abs;

public class ZoneUtilities {
    private static final double LOOKAHEAD_TIME = 0.5;

    private static final Debouncer TRENCH_DEBOUNCER = new Debouncer(0.1, Debouncer.DebounceType.kBoth);
    private static final Debouncer AREA_DEBOUNCER = new Debouncer(0.1, Debouncer.DebounceType.kBoth);

    public static boolean willBeInTrench() {
        return checkZone(TRENCH_DEBOUNCER, ZoneUtilities::isInTrench);
    }

    public static boolean willBeInTrenchArea() {
        return checkZone(AREA_DEBOUNCER, ZoneUtilities::isInTrenchArea);
    }

    private static boolean checkZone(Debouncer debouncer, Predicate<Translation2d> zone) {
        final Translation2d current = POSE_ESTIMATOR.getPose().getTranslation();
        final Translation2d future = POSE_ESTIMATOR.predictFuturePose(LOOKAHEAD_TIME).getTranslation();
        final Translation2d mid = current.plus(future).times(0.5);

        return debouncer.calculate(zone.test(current) || zone.test(future) || zone.test(mid));
    }

    private static boolean isInTrench(Translation2d pose) {
        return BOTTOM_TRENCH.contains(fold(pose));
    }

    private static boolean isInTrenchArea(Translation2d pose) {
        return BOTTOM_TRENCH_AREA.contains(fold(pose));
    }

    private static Translation2d fold(Translation2d pose) {
        return new Translation2d(
                HALF_FIELD_LENGTH - abs(pose.getX() - HALF_FIELD_LENGTH),
                HALF_FIELD_WIDTH - abs(pose.getY() - HALF_FIELD_WIDTH)
        );
    }
}