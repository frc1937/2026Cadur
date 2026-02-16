package frc.lib.math;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;

import java.util.Map;
import java.util.function.Supplier;

public class TimeAdjustedTransform {
    private final TimeInterpolatableBuffer<Rotation2d> angleBuffer;
    private final Pose3d mechanismOrigin;
    private final Supplier<Rotation2d> fallbackAngle;

    private double latestVelocityRPS = 0.0;

    public TimeAdjustedTransform(double bufferSec, Pose3d mechanismOrigin, Supplier<Rotation2d> fallbackAngle) {
        this.angleBuffer = TimeInterpolatableBuffer.createBuffer(Rotation2d::interpolate, bufferSec);
        this.mechanismOrigin = mechanismOrigin;
        this.fallbackAngle = fallbackAngle;
    }

    public void update(Rotation2d position, double timestamp, double currentVelocityRPS) {
        angleBuffer.addSample(timestamp, position);
        latestVelocityRPS = currentVelocityRPS;
    }

    public Transform3d getRobotToCamera(double timestamp, Transform3d mechanismToCameraTransform) {
        final Rotation2d angle = calculateAngleAtTime(timestamp);

        final Transform3d rotationTransform = new Transform3d(
                Translation3d.kZero,
                new Rotation3d(0, 0, angle.getRadians())
        );

        final Pose3d rotatedOrigin = mechanismOrigin.plus(rotationTransform);
        final Pose3d cameraPose = rotatedOrigin.plus(mechanismToCameraTransform);

        return cameraPose.minus(Pose3d.kZero);
    }

    private Rotation2d calculateAngleAtTime(double timestamp) {
        final Rotation2d sampledAngle = angleBuffer.getSample(timestamp).orElse(null);

        if (sampledAngle == null)
            return estimateFutureAngle(timestamp);

        return sampledAngle;
    }

    private Rotation2d estimateFutureAngle(double futureTimestamp) {
        final Map.Entry<Double, Rotation2d> latestEntry = angleBuffer.getInternalBuffer().lastEntry();

        if (latestEntry == null)
            return fallbackAngle.get();

        final Double latestTimestamp = latestEntry.getKey();
        final Rotation2d latestAngle = latestEntry.getValue();

        final double timeDeltaSeconds = futureTimestamp - latestTimestamp;
        final double predictedRotations = latestVelocityRPS * timeDeltaSeconds;

        return latestAngle.plus(Rotation2d.fromRotations(predictedRotations));
    }
}