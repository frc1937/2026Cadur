package frc.lib.util.flippable;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * A class that represents a {@link Pose2d} that can be flipped when the robot is on the red alliance.
 */
public class FlippablePose2d extends Flippable<Pose2d> {
    /**
     * Creates a new FlippablePose2d from x, y, and rotation components.
     *
     * @param x                         the x value of the pose
     * @param y                         the y value of the pose
     * @param rotation                  the rotation of the pose
     * @param shouldFlipWhenRedAlliance whether to flip when on the red alliance
     */
    public FlippablePose2d(double x, double y, Rotation2d rotation, boolean shouldFlipWhenRedAlliance) {
        this(new Pose2d(x, y, rotation), shouldFlipWhenRedAlliance);
    }

    /**
     * Creates a new FlippablePose2d from a translation and rotation.
     *
     * @param translation2d             the translation of the pose
     * @param rotation                  the rotation of the pose
     * @param shouldFlipWhenRedAlliance whether to flip when on the red alliance
     */
    public FlippablePose2d(Translation2d translation2d, Rotation2d rotation, boolean shouldFlipWhenRedAlliance) {
        this(new Pose2d(translation2d, rotation), shouldFlipWhenRedAlliance);
    }

    /**
     * Creates a new FlippablePose2d from a translation and a rotation in radians.
     *
     * @param translation2d             the translation of the pose
     * @param rotationRadians           the rotation of the pose in radians
     * @param shouldFlipWhenRedAlliance whether to flip when on the red alliance
     */
    public FlippablePose2d(Translation2d translation2d, double rotationRadians, boolean shouldFlipWhenRedAlliance) {
        this(new Pose2d(translation2d, new Rotation2d(rotationRadians)), shouldFlipWhenRedAlliance);
    }

    /**
     * Creates a new FlippablePose2d from a {@link Pose2d}.
     *
     * @param nonFlippedPose            the pose when on the blue alliance
     * @param shouldFlipWhenRedAlliance whether to flip when on the red alliance
     */
    public FlippablePose2d(Pose2d nonFlippedPose, boolean shouldFlipWhenRedAlliance) {
        super(nonFlippedPose, shouldFlipWhenRedAlliance);
    }

    /**
     * Creates a new FlippablePose2d with independent X and Y flip control.
     *
     * @param nonFlippedPose the pose when on the blue alliance
     * @param shouldFlipX    whether to flip about the X axis when on the red alliance
     * @param shouldFlipY    whether to flip about the Y axis when on the red alliance
     */
    public FlippablePose2d(Pose2d nonFlippedPose, boolean shouldFlipX, boolean shouldFlipY) {
        super(nonFlippedPose, shouldFlipX, shouldFlipY);
    }

    /**
     * Returns the rotation of this pose as a {@link FlippableRotation2d}, respecting the same flip mode.
     *
     * @return the rotation of the pose
     */
    public FlippableRotation2d getRotation() {
        return new FlippableRotation2d(nonFlippedObject.getRotation(), shouldFlipX, shouldFlipY);
    }

    @Override
    protected Pose2d flip(Pose2d pose) {
        return FlippingUtil.flipFieldPose(pose);
    }

    @Override
    protected Pose2d xFlip(Pose2d pose) {
        return FlippableUtils.flipAboutXAxis(pose);
    }

    @Override
    protected Pose2d yFlip(Pose2d pose) {
        return FlippableUtils.flipAboutYAxis(pose);
    }
}