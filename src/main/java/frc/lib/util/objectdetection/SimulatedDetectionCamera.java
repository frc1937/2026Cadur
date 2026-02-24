package frc.lib.util.objectdetection;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import static frc.robot.RobotContainer.POSE_ESTIMATOR;

public class SimulatedDetectionCamera extends DetectionCamera {
    private static final Rotation2d HORIZONTAL_FOV = Rotation2d.fromDegrees(75);
    private static final double MAX_DISTANCE_METERS = 5, MIN_DISTANCE_METERS = 0.05;
    private final List<PhotonTrackedTarget> detectedTargets = new ArrayList<>();

    private static final List<TargetCorner> FAKE_CORNERS = List.of(
            new TargetCorner(0, 0), new TargetCorner(0, 0),
            new TargetCorner(0, 0), new TargetCorner(0, 0)
    );

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
    private long sequenceID = 0;

    public SimulatedDetectionCamera(String name, Transform3d robotToCamera) {
        super(name);
        this.robotToCamera = robotToCamera;
    }

    @Override
    protected void refreshInputs(DetectionCameraInputsAutoLogged inputs) {
        inputs.avgTargetYaw = 0xCAFEBABE;
        inputs.avgTargetPitch = 0xCAFEBABE;
        inputs.targetYaws = new  ArrayList<>();
        inputs.targetPitches = new  ArrayList<>();
        latestTargets = List.of();

        if (robotToCamera == null) return;

        final PhotonPipelineResult result = getSimulatedResults();
        if (!result.hasTargets()) return;

        latestTargets = result.getTargets();

        final PhotonCluster bestCluster = ClusterHandler.getBestCluster(latestTargets);
        if (bestCluster == null) return;

        final PhotonTrackedTarget bestTarget = bestCluster.getClosestTarget();
        inputs.avgTargetYaw = bestTarget.getYaw();
        inputs.avgTargetPitch = bestTarget.getPitch();
        inputs.targetYaws = bestCluster.getYaws();
        inputs.targetPitches = bestCluster.getPitches();
    }

    private PhotonPipelineResult getSimulatedResults() {
        detectedTargets.clear();
        final Pose3d cameraPose = new Pose3d(POSE_ESTIMATOR.getPose()).transformBy(robotToCamera);

        for (Translation2d ball : FIELD_OBJECTS) {
            final double distance = ball.getDistance(cameraPose.toPose2d().getTranslation());

            if (distance < MIN_DISTANCE_METERS || distance > MAX_DISTANCE_METERS) continue;

            final Rotation2d angleToTarget = ball.minus(cameraPose.toPose2d().getTranslation()).getAngle();
            final Rotation2d relativeYaw = angleToTarget.minus(cameraPose.toPose2d().getRotation());

            if (Math.abs(relativeYaw.getDegrees()) <= HORIZONTAL_FOV.getDegrees() / 2.0) {
                final double deltaZ = -cameraPose.getZ();
                final double pitch = Math.toDegrees(Math.atan2(deltaZ, distance));
                final double area = Math.min(100.0, 0.5 / Math.pow(distance, 2));

                detectedTargets.add(new PhotonTrackedTarget(
                        relativeYaw.getDegrees(), pitch, area, 0.0,
                        -1, 0, 0.95f,
                        new Transform3d(), new Transform3d(), 0.0,
                        FAKE_CORNERS, new ArrayList<>()
                ));
            }
        }

        final long timestampMicros = (long) (Timer.getFPGATimestamp() * 1e6);
        return new PhotonPipelineResult(
                sequenceID++,
                timestampMicros,
                timestampMicros,
                0,
                List.copyOf(detectedTargets),
                Optional.empty()
        );
    }
}