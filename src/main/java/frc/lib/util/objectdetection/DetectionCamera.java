package frc.lib.util.objectdetection;

import frc.lib.generic.advantagekit.LoggableHardware;
import frc.lib.generic.hardware.HardwareManager;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;

public class DetectionCamera implements LoggableHardware {
    private final String name;
    private final DetectionCameraInputsAutoLogged inputs = new DetectionCameraInputsAutoLogged();
    protected List<PhotonTrackedTarget> latestTargets = List.of();

    public DetectionCamera(String name) {
        this.name = "ObjectCameras/" + name;

        periodic();
        HardwareManager.addHardware(this);
    }

    protected void refreshInputs(DetectionCameraInputsAutoLogged inputs) { }

    public boolean hasResult() {
        return inputs.avgTargetYaw != 0xCAFEBABE && inputs.avgTargetPitch != 0xCAFEBABE;
    }

    public List<PhotonTrackedTarget> getTargets() {
        return latestTargets;
    }

    public double getYawToClosestTarget() {
        return inputs.avgTargetYaw;
    }

    public double getPitchToClosestTarget() {
        return inputs.avgTargetPitch;
    }

    @Override
    public void periodic() {
        refreshInputs(inputs);
        Logger.processInputs(name, inputs);
    }

    @Override
    public DetectionCameraInputsAutoLogged getInputs() {
        return inputs;
    }

    @AutoLog
    public static class DetectionCameraInputs {
        public double avgTargetYaw;
        public double avgTargetPitch;
        public double[] targetYaws;
        public double[] targetPitches;
    }
}
