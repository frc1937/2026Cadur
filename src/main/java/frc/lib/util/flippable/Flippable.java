package frc.lib.util.flippable;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.Optional;

/**
 * A class that allows for objects to be flipped across the center of the field when the robot is on the red alliance.
 * This is useful for placing field elements and other objects that are flipped across the field, or for flipping the target heading to face a field element.
 * This either represents a field mirrored vertically over the center of the field, or a field that is rotationally symmetric:
 * the red alliance side is the blue alliance side rotated by 180 degrees.
 * The code will automatically determine which type of field the robot is on and flip the object accordingly.
 *
 * <p>Three flip modes are supported:
 * <ul>
 *   <li>X-axis flip only: pass {@code shouldFlipX=true, shouldFlipY=false}</li>
 *   <li>Y-axis flip only: pass {@code shouldFlipX=false, shouldFlipY=true}</li>
 *   <li>Full field flip (both axes): pass {@code shouldFlipX=true, shouldFlipY=true}, or use the convenience constructor</li>
 * </ul>
 *
 * @param <T> the type of object to flip
 */
public abstract class Flippable<T> {
    private static final Timer UPDATE_ALLIANCE_TIMER = new Timer();
    private static boolean IS_RED_ALLIANCE = notCachedIsRedAlliance();

    protected final T nonFlippedObject, flippedObject, xFlippedObject, yFlippedObject;
    protected final boolean shouldFlipX, shouldFlipY, shouldFlipBoth;

    /**
     * Initializes the Flippable class. This should be called once in RobotContainer.
     */
    public static void init() {
        UPDATE_ALLIANCE_TIMER.start();

        new Trigger(() -> UPDATE_ALLIANCE_TIMER.advanceIfElapsed(1))
                .onTrue(getUpdateAllianceCommand());
    }

    /**
     * @return whether the robot is on the red alliance, cached and refreshed every second
     */
    public static boolean isRedAlliance() {
        return IS_RED_ALLIANCE;
    }

    /**
     * @return a command that refreshes the cached alliance value; runs while disabled
     */
    private static Command getUpdateAllianceCommand() {
        return new InstantCommand(() -> IS_RED_ALLIANCE = notCachedIsRedAlliance()).ignoringDisable(true);
    }

    /**
     * @return whether the robot is on the red alliance, queried directly from the DriverStation (not cached)
     */
    private static boolean notCachedIsRedAlliance() {
        final Optional<DriverStation.Alliance> optionalAlliance = DriverStation.getAlliance();
        return optionalAlliance.orElse(DriverStation.Alliance.Red).equals(DriverStation.Alliance.Red);
    }

    /**
     * Creates a new Flippable with independent X and Y flip control.
     * Only the variants that can actually be selected are computed eagerly:
     * <ul>
     *   <li>If both flags are true, only the full-flip variant is computed.</li>
     *   <li>If only one flag is true, only that axis-flip variant is computed.</li>
     *   <li>If neither flag is true, no flip variants are computed.</li>
     * </ul>
     *
     * @param nonFlippedObject the object when the robot is on the blue alliance
     * @param shouldFlipX      whether to flip about the X axis when on the red alliance
     * @param shouldFlipY      whether to flip about the Y axis when on the red alliance
     */
    protected Flippable(T nonFlippedObject, boolean shouldFlipX, boolean shouldFlipY) {
        this.nonFlippedObject = nonFlippedObject;

        this.shouldFlipX = shouldFlipX;
        this.shouldFlipY = shouldFlipY;
        this.shouldFlipBoth = shouldFlipX && shouldFlipY;

        this.flippedObject = shouldFlipBoth ? flip(nonFlippedObject) : nonFlippedObject;
        this.xFlippedObject = (shouldFlipX && !shouldFlipBoth) ? xFlip(nonFlippedObject) : nonFlippedObject;
        this.yFlippedObject = (shouldFlipY && !shouldFlipBoth) ? yFlip(nonFlippedObject) : nonFlippedObject;
    }

    /**
     * Convenience constructor for the common case of a standard full-field flip.
     * Equivalent to {@code Flippable(nonFlippedObject, shouldFlip, shouldFlip)}.
     *
     * @param nonFlippedObject the object when the robot is on the blue alliance
     * @param shouldFlip       whether to apply a full-field flip when on the red alliance
     */
    protected Flippable(T nonFlippedObject, boolean shouldFlip) {
        this(nonFlippedObject, shouldFlip, shouldFlip);
    }

    /**
     * Returns the appropriate variant of the object for the current alliance.
     * If the robot is not on the red alliance, the non-flipped object is always returned.
     *
     * @return the current object, flipped if necessary
     */
    public T get() {
        if (!IS_RED_ALLIANCE) return nonFlippedObject;

        if (shouldFlipBoth) return flippedObject;
        if (shouldFlipX) return xFlippedObject;
        if (shouldFlipY) return yFlippedObject;

        return nonFlippedObject;
    }

    /**
     * Performs a full field flip of the object (handles both mirror and rotationally symmetric fields).
     *
     * @param object the object to flip
     * @return the flipped object
     */
    protected abstract T flip(T object);

    /**
     * Flips the object about the X axis.
     * Subclasses must override this method if they are constructed with {@code shouldFlipX=true, shouldFlipY=false}.
     *
     * @param object the object to flip
     * @return the X-flipped object
     * @throws UnsupportedOperationException if not implemented by the subclass
     */
    protected abstract T xFlip(T object);

    /**
     * Flips the object about the Y axis.
     * Subclasses must override this method if they are constructed with {@code shouldFlipX=false, shouldFlipY=true}.
     *
     * @param object the object to flip
     * @return the Y-flipped object
     * @throws UnsupportedOperationException if not implemented by the subclass
     */
    protected abstract T yFlip(T object);
}