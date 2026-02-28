package frc.robot.utilities;

import edu.wpi.first.math.geometry.Translation2d;

public class Zone {
    protected final double xMin, xMax, yMin, yMax;

    public Zone(double xMin, double xMax, double yMin, double yMax) {
        this.xMin = xMin;
        this.xMax = xMax;
        this.yMin = yMin;
        this.yMax = yMax;
    }

    public boolean contains(Translation2d pose) {
        return pose.getX() >= xMin && pose.getX() <= xMax && pose.getY() >= yMin && pose.getY() <= yMax;
    }

    public boolean contains(double x, double y) {
        return x >= xMin && x <= xMax && y >= yMin && y <= yMax;
    }

    public Zone mirroredX() {
        return new Zone(
                FieldConstants.FIELD_LENGTH - xMax,
                FieldConstants.FIELD_LENGTH - xMin,
                yMin,
                yMax);
    }

    public Zone mirroredY() {
        return new Zone(
                xMin,
                xMax,
                FieldConstants.FIELD_WIDTH - yMax,
                FieldConstants.FIELD_WIDTH - yMin);
    }

    /** list of corners, with the bottom left corner repeated at the end to form a closed loop */
    public Translation2d[] getCorners() {
        return new Translation2d[] {
                new Translation2d(xMin, yMin),
                new Translation2d(xMax, yMin),
                new Translation2d(xMax, yMax),
                new Translation2d(xMin, yMax),
                new Translation2d(xMin, yMin)
        };
    }
}
