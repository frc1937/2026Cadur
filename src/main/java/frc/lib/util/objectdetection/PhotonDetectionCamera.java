package frc.lib.util.objectdetection;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
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
        inputs.avgTargetYaw = 0xCAFEBABE;
        inputs.avgTargetPitch = 0xCAFEBABE;
        inputs.targetYaws = new double[0];
        inputs.targetPitches = new double[0];
        latestTargets = List.of();

        if (camera == null || !camera.isConnected())  return;

        final List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        if (results.isEmpty()) return;

        final PhotonPipelineResult latestResult = results.get(results.size() - 1);
        if (!latestResult.hasTargets()) return;

        latestTargets = latestResult.getTargets();

        final PhotonCluster bestCluster = ClusterHandler.getBestCluster(latestTargets);
        if (bestCluster == null) return;

        inputs.avgTargetYaw = bestCluster.getAvgYaw();
        inputs.avgTargetPitch = bestCluster.getAvgPitch();
        inputs.targetYaws = bestCluster.getYaws();
        inputs.targetPitches = bestCluster.getPitches();
    }
}
