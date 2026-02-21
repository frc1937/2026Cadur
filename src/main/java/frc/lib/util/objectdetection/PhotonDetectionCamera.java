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
        latestTargets = List.of();

        if (camera == null || !camera.isConnected())  return;

        final List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        if (results.isEmpty()) return;

        final PhotonPipelineResult latestResult = results.get(results.size() - 1);
        if (!latestResult.hasTargets()) return;

        latestTargets = latestResult.getTargets();

        final PhotonTrackedTarget bestTarget = PhotonTargetClusterer.getBestCluster(latestTargets).getClosestTarget();
        inputs.closestTargetYaw = bestTarget.getYaw();
        inputs.closestTargetPitch = bestTarget.getPitch();
    }
}
