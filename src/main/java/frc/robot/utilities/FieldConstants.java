package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.util.flippable.FlippablePose2d;
import frc.lib.util.flippable.FlippableTranslation2d;
import frc.lib.util.flippable.FlippableTranslation3d;

import static frc.robot.RobotContainer.POSE_ESTIMATOR;

public class FieldConstants {
    public enum TowerLevel {
        L1(0.1),
        L2(0.2),
        L3(0.3);

        public final double height;

        TowerLevel(double height) {
            this.height = height;
        }

        public double getHeight() {
            return this.height;
        }
    }

    public static final double
            FIELD_WIDTH = 8.07, HALF_FIELD_WIDTH = FIELD_WIDTH / 2,
            FIELD_LENGTH = 16.54, HALF_FIELD_LENGTH = FIELD_LENGTH / 2,
            HUB_SIZE = 1.1938, HALF_HUB_SIZE = HUB_SIZE / 2;

    public static final FlippableTranslation3d HUB_TOP_POSITION
            = new FlippableTranslation3d(4.604766, 4.0215, 1.8288, true);

    public static final Translation2d
            LEFT_PASSING_POINT = new Translation2d(0.25 * HUB_TOP_POSITION.get().getX(), HUB_TOP_POSITION.get().getY() - 0.7),
            RIGHT_PASSING_POINT = new Translation2d(0.25 * HUB_TOP_POSITION.get().getX(), HUB_TOP_POSITION.get().getY() + 0.7);

    public static final FlippableTranslation2d TOWER_POSITION = new FlippableTranslation2d(3.730244, 1.016, true);

    public enum Trench {
        BLUE_BOTTOM_TRENCH_CENTER(new FlippableTranslation2d(4.604766, 0.6395, false)),
        BLUE_TOP_TRENCH_CENTER(new FlippableTranslation2d(4.604766, FIELD_WIDTH - 0.6395, false));

        final FlippableTranslation2d trenchPose;

        Trench(FlippableTranslation2d trenchPose) {
            this.trenchPose = trenchPose;
        }

        public Translation2d get() {
            return trenchPose.get();
        }

        public static Trench getClosestTrenchToRobot() {
            return  (POSE_ESTIMATOR.getPose().getY() - HALF_FIELD_WIDTH) <= 0 ? BLUE_BOTTOM_TRENCH_CENTER : BLUE_TOP_TRENCH_CENTER;
        }
    }

    public static final Zone BOTTOM_TRENCH = new Zone(3.770766, 5.438766, 0, 1.279);
    public static final Zone BOTTOM_TRENCH_AREA = new Zone(3.770766-0.5, 5.438766+0.5, 0, 1.279);

    public static final Zone BLUE_ALLIANCE_ZONE = new Zone(0, 4.604766, 0, FIELD_WIDTH);
    public static final Zone RED_ALLIANCE_ZONE = BLUE_ALLIANCE_ZONE.mirroredX();

    public static final FlippablePose2d BALLS_BOTTOM_START = new FlippablePose2d(new Pose2d(7.824, 1.257, Rotation2d.k180deg), false, true);
    public static final FlippablePose2d BALLS_TOP_START = new FlippablePose2d(new Pose2d(7.824, FIELD_WIDTH-1.257, Rotation2d.k180deg), false, true);
    public static final FlippableTranslation2d BALLS_MIDDLE = new FlippableTranslation2d(7.824, HALF_FIELD_WIDTH, true);

    public static final FlippableTranslation2d DEPOT_LOCATION = new FlippableTranslation2d(0.69, 5.96, true);
    public static final FlippableTranslation2d OUTPOST_LOCATION = new FlippableTranslation2d(0.71, 0.67, true);

}