package frc.lib.util.flippable;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * A class that represents a {@link Translation3d} that can be flipped when the robot is on the red alliance.
 * The Z value will have no change.
 */
public class FlippableTranslation3d extends Flippable<Translation3d> {
    /**
     * Creates a new FlippableTranslation3d with the given x, y, and z values.
     *
     * @param x                         the x value of the translation
     * @param y                         the y value of the translation
     * @param z                         the z value of the translation
     * @param shouldFlipWhenRedAlliance should the position be flipped when the robot is on the red alliance
     */
    public FlippableTranslation3d(double x, double y, double z, boolean shouldFlipWhenRedAlliance) {
        this(new Translation3d(x, y, z), shouldFlipWhenRedAlliance);
    }

    public FlippableTranslation3d(double x, double y, double z, boolean shouldFlipX, boolean shouldFlipY) {
        this(new Translation3d(x,y,z), shouldFlipX, shouldFlipY);
    }

    /**
     * Creates a new FlippableTranslation3d with the given translation.
     *
     * @param nonFlippedTranslation     the translation to flip
     * @param shouldFlipWhenRedAlliance should the position be flipped when the robot is on the red alliance
     */
    public FlippableTranslation3d(Translation3d nonFlippedTranslation, boolean shouldFlipWhenRedAlliance) {
        super(nonFlippedTranslation, shouldFlipWhenRedAlliance);
    }

    /**
     * Creates a new FlippableTranslation3d with independent X and Y flip control.
     *
     * @param nonFlippedTranslation the translation when on the blue alliance
     * @param shouldFlipX           whether to flip about the X axis when on the red alliance
     * @param shouldFlipY           whether to flip about the Y axis when on the red alliance
     */
    public FlippableTranslation3d(Translation3d nonFlippedTranslation, boolean shouldFlipX, boolean shouldFlipY) {
        super(nonFlippedTranslation, shouldFlipX, shouldFlipY);
    }

    /**
     * Performs a full field flip of the XY plane. The Z value is preserved.
     *
     * @param translation the translation to flip
     * @return the flipped translation with Z unchanged
     */
    @Override
    protected Translation3d flip(Translation3d translation) {
        final Translation2d flipped = FlippingUtil.flipFieldPosition(translation.toTranslation2d());
        return new Translation3d(flipped.getX(), flipped.getY(), translation.getZ());
    }

    /**
     * Flips the translation about the X axis. The Z value is preserved.
     *
     * @param translation the translation to flip
     * @return the X-flipped translation with Z unchanged
     */
    @Override
    protected Translation3d xFlip(Translation3d translation) {
        final Translation2d flipped = FlippableUtils.flipAboutXAxis(translation.toTranslation2d());
        return new Translation3d(flipped.getX(), flipped.getY(), translation.getZ());
    }

    /**
     * Flips the translation about the Y axis. The Z value is preserved.
     *
     * @param translation the translation to flip
     * @return the Y-flipped translation with Z unchanged
     */
    @Override
    protected Translation3d yFlip(Translation3d translation) {
        final Translation2d flipped = FlippableUtils.flipAboutYAxis(translation.toTranslation2d());
        return new Translation3d(flipped.getX(), flipped.getY(), translation.getZ());
    }
}