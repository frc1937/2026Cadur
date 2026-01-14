package frc.lib.generic.visualization.mechanisms;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class MechanismConstants {
    protected static final int
            DEFAULT_ROOT_X = 5,
            DEFAULT_ROOT_Y = 5,
            DEFAULT_CANVAS_WIDTH = 10,
            DEFAULT_CANVAS_HEIGHT = 10,
            DEFAULT_LINE_WIDTH = 3;

    protected static final double
            SLOWEST_SPEED_TO_CONSIDER_IDLE_RPS = 0.001,
            RPS_TO_LENGTH_FACTOR = 25,
            DEFAULT_ARM_ANGLE = 0;

    protected static final double
            ARROW_TOP_ANGLE = 135,
            ARROW_BOTTOM_ANGLE = -135,
            ARROW_TOP_ANGLE_INVERSE = 45,
            ARROW_BOTTOM_ANGLE_INVERSE = -45;

    protected static final Color8Bit
            RED = new Color8Bit(Color.kRed),
            GREEN = new Color8Bit(Color.kGreen),
            BLUE = new Color8Bit(Color.kBlue),
            GRAY = new Color8Bit(Color.kGray),
            DARK_BLUE = new Color8Bit(Color.kDarkBlue),
            DARK_GRAY = new Color8Bit(Color.kDarkGray);
}