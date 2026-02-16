package frc.lib.util.objectdetection;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;

public class PhotonDetectionCamera extends DetectionCamera {
    private final PhotonCamera camera;

    public PhotonDetectionCamera(String name) {
        super(name);

        PhotonCamera.setVersionCheckEnabled(false);
        camera = new PhotonCamera(name);
    }

    @Override
    protected void refreshInputs(DetectionCameraInputsAutoLogged inputs) {
        inputs.closestTargetYaw = 0xCAFEBABE;
        inputs.closestTargetPitch = 0xCAFEBABE;

        if (camera == null || !camera.isConnected())  return;

        final List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        if (results.isEmpty()) return;

        final PhotonPipelineResult latestResult = results.get(results.size() - 1);
        if (!latestResult.hasTargets()) return;

        final List<PhotonTrackedTarget> targets = latestResult.getTargets();
        inputs.targets = targets;

        PhotonTrackedTarget bestTarget = null;
        double lowestScore = Double.MAX_VALUE;

        for (PhotonTrackedTarget target : targets) {
            final double score = Math.pow(target.getYaw(), 2) + Math.pow(target.getPitch(), 2);

            if (score < lowestScore) {
                lowestScore = score;
                bestTarget = target;
            }
        }

        if (bestTarget != null) {
            inputs.closestTargetYaw = bestTarget.getYaw();
            inputs.closestTargetPitch = bestTarget.getPitch();
        }
    }
}
