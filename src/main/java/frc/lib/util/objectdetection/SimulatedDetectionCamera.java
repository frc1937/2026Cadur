package frc.lib.util.objectdetection;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;

import static frc.robot.RobotContainer.POSE_ESTIMATOR;

public class SimulatedDetectionCamera extends DetectionCamera {
    private static final Rotation2d HORIZONTAL_FOV = Rotation2d.fromDegrees(75);
    private static final double MAX_DISTANCE_METERS = 5, MIN_DISTANCE_METERS = 0.05;

    private static final Translation2d[] FIELD_OBJECTS = new Translation2d[]{
            new Translation2d(2.9, 7),
            new Translation2d(2.9, 5.5),
            new Translation2d(2.9, 4.1),
            new Translation2d(8.3, 7.45),
            new Translation2d(8.3, 5.75),
            new Translation2d(8.3, 4.1),
            new Translation2d(8.3, 2.45),
            new Translation2d(8.3, 0.75),
            new Translation2d(13.65, 7),
            new Translation2d(13.65, 5.5),
            new Translation2d(13.65, 4.1)
    };

    private final Transform3d robotToCamera;

    public SimulatedDetectionCamera(String name, Transform3d robotToCamera) {
        super(name);
        this.robotToCamera = robotToCamera;
    }

    @Override
    protected void refreshInputs(DetectionCameraInputsAutoLogged inputs) {
        if (robotToCamera == null) return;

        inputs.closestTargetYaw = 0xCAFEBABE;
        inputs.closestTargetPitch = 0xCAFEBABE;

        final Pose3d cameraPose = new Pose3d(POSE_ESTIMATOR.getCurrentPose()).transformBy(robotToCamera);

        Translation2d bestTarget = getBestTarget(cameraPose);

        if (bestTarget != null) {
            final Rotation2d finalYaw = bestTarget.minus(cameraPose.toPose2d().getTranslation()).getAngle()
                    .minus(cameraPose.toPose2d().getRotation());

            final double groundDist = bestTarget.getDistance(cameraPose.toPose2d().getTranslation());
            final double deltaZ = -cameraPose.getZ();
            final double finalPitch = Math.toDegrees(Math.atan2(deltaZ, groundDist));

            inputs.closestTargetYaw = finalYaw.getDegrees();
            inputs.closestTargetPitch = finalPitch;
        }
    }

    private static Translation2d getBestTarget(Pose3d cameraPose) {
        double bestScore = Double.POSITIVE_INFINITY;
        Translation2d bestTarget = null;

        for (Translation2d ball : FIELD_OBJECTS) {
            double dist = ball.getDistance(cameraPose.toPose2d().getTranslation());

            if (dist < MIN_DISTANCE_METERS || dist > MAX_DISTANCE_METERS) continue;

            final Rotation2d angleToTarget = ball.minus(cameraPose.toPose2d().getTranslation()).getAngle();
            final Rotation2d relativeYaw = angleToTarget.minus(cameraPose.toPose2d().getRotation());

            if (Math.abs(relativeYaw.getDegrees()) > HORIZONTAL_FOV.getDegrees() / 2.0) continue;

            if (dist < bestScore) {
                bestScore = dist;
                bestTarget = ball;
            }
        }

        return bestTarget;
    }
}
