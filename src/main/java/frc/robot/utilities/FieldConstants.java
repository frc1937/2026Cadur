package frc.robot.utilities;

import frc.lib.util.flippable.FlippableTranslation2d;
import frc.lib.util.flippable.FlippableTranslation3d;

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
            FIELD_WIDTH = 8.07,
            FIELD_LENGTH = 16.54;

    public static final FlippableTranslation3d HUB_POSITION = new FlippableTranslation3d(4.034536, 4.625594, 1.8288, true);
    public static final FlippableTranslation2d TOWER_POSITION = new FlippableTranslation2d(3.730244, 1.016, true);
}