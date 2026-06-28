package frc.robot.utilities;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Translation2d;
import org.littletonrobotics.junction.Logger;

import java.util.function.Predicate;

import static frc.lib.util.flippable.Flippable.isRedAlliance;
import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.utilities.FieldConstants.*;
import static java.lang.Math.abs;

public class ZoneUtilities {
    private static final double LOOKAHEAD_TIME_SECONDS = 0.01;

    private static final Debouncer TRENCH_DEBOUNCER = new Debouncer(0.1, Debouncer.DebounceType.kBoth);
    private static final Debouncer TRENCH_AREA_DEBOUNCER = new Debouncer(0.1, Debouncer.DebounceType.kBoth);
    private static final Debouncer ALLIANCE_ZONE_DEBOUNCER = new Debouncer(0.1, Debouncer.DebounceType.kBoth);
    private static final Debouncer OPPOSITE_ZONE_DEBOUNCER = new Debouncer(0.1, Debouncer.DebounceType.kBoth);

    public static boolean isInTrench() {
        final boolean check = checkZone(TRENCH_DEBOUNCER, ZoneUtilities::testInTrench);
        Logger.recordOutput("Zone/isInTrench", check);
        return check;
    }

    public static boolean isInTrenchArea() {
        final boolean check = checkZone(TRENCH_AREA_DEBOUNCER, ZoneUtilities::testInTrenchArea);
        Logger.recordOutput("Zone/isInTrenchArea", check);
        return check;
    }

    public static boolean isInAllianceZone() {
        final boolean check = checkZone(ALLIANCE_ZONE_DEBOUNCER, ZoneUtilities::testInAllianceZone);
        Logger.recordOutput("Zone/isInAllianceZone", check);
        return check;
    }

    public static boolean isInOppositeAllianceZone() {
        final boolean check = checkZone(OPPOSITE_ZONE_DEBOUNCER, ZoneUtilities::testInOppositeAllianceZone);
        Logger.recordOutput("Zone/isInOppositeAllianceZone", check);
        return check;
    }

    private static boolean checkZone(Debouncer debouncer, Predicate<Translation2d> zone) {
        Translation2d current = POSE_ESTIMATOR.getPose().getTranslation();
        Translation2d future = POSE_ESTIMATOR.predictFuturePose(LOOKAHEAD_TIME_SECONDS).getTranslation();
        Translation2d mid = current.interpolate(future, 0.5);

        return debouncer.calculate(zone.test(current) || zone.test(mid) || zone.test(future));
    }

    private static boolean testInAllianceZone(Translation2d pose) {
        return (isRedAlliance() ? RED_ALLIANCE_ZONE : BLUE_ALLIANCE_ZONE).contains(pose);
    }

    private static boolean testInOppositeAllianceZone(Translation2d pose) {
        return (isRedAlliance() ? BLUE_ALLIANCE_ZONE : RED_ALLIANCE_ZONE).contains(pose);
    }

    private static boolean testInTrench(Translation2d pose) {
        return BOTTOM_TRENCH.contains(fold(pose));
    }

    private static boolean testInTrenchArea(Translation2d pose) {
        return BOTTOM_TRENCH_AREA.contains(fold(pose));
    }

    private static Translation2d fold(Translation2d pose) {
        return new Translation2d(
                HALF_FIELD_LENGTH - abs(pose.getX() - HALF_FIELD_LENGTH),
                HALF_FIELD_WIDTH - abs(pose.getY() - HALF_FIELD_WIDTH)
        );
    }
}