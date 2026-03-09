package frc.lib.util.flippable;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * A class that represents a {@link Translation2d} that can be flipped when the robot is on the red alliance.
 */
public class FlippableTranslation2d extends Flippable<Translation2d> {
    /**
     * Creates a new FlippableTranslation2d with the given x and y values.
     *
     * @param x                         the x value of the translation
     * @param y                         the y value of the translation
     * @param shouldFlipWhenRedAlliance should the position be flipped when the robot is on the red alliance
     */
    public FlippableTranslation2d(double x, double y, boolean shouldFlipWhenRedAlliance) {
        this(new Translation2d(x, y), shouldFlipWhenRedAlliance);
    }

    public FlippableTranslation2d(double x, double y, boolean shouldFlipX, boolean shouldFlipY) {
        this(new Translation2d(x, y), shouldFlipX, shouldFlipY);
    }

    /**
     * Creates a new FlippableTranslation2d with the given translation.
     *
     * @param nonFlippedTranslation     the translation to flip
     * @param shouldFlipWhenRedAlliance should the position be flipped when the robot is on the red alliance
     */
    public FlippableTranslation2d(Translation2d nonFlippedTranslation, boolean shouldFlipWhenRedAlliance) {
        super(nonFlippedTranslation, shouldFlipWhenRedAlliance);
    }

    /**
     * Creates a new FlippableTranslation2d with independent X and Y flip control.
     *
     * @param nonFlippedTranslation the translation when on the blue alliance
     * @param shouldFlipX           whether to flip about the X axis when on the red alliance
     * @param shouldFlipY           whether to flip about the Y axis when on the red alliance
     */
    public FlippableTranslation2d(Translation2d nonFlippedTranslation, boolean shouldFlipX, boolean shouldFlipY) {
        super(nonFlippedTranslation, shouldFlipX, shouldFlipY);
    }

    @Override
    protected Translation2d flip(Translation2d translation) {
        return FlippingUtil.flipFieldPosition(translation);
    }

    @Override
    protected Translation2d xFlip(Translation2d translation) {
        return FlippableUtils.flipAboutXAxis(translation);
    }

    @Override
    protected Translation2d yFlip(Translation2d translation) {
        return FlippableUtils.flipAboutYAxis(translation);
    }
}
