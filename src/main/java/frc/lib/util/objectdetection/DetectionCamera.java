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

    public DetectionCamera(String name) {
        this.name = "ObjectCameras/" + name;

        periodic();
        HardwareManager.addHardware(this);
    }

    protected void refreshInputs(DetectionCameraInputsAutoLogged inputs) { }

    public boolean hasResult() {
        return inputs.closestTargetYaw != 0xCAFEBABE && inputs.closestTargetPitch != 0xCAFEBABE;
    }

    public double getYawToClosestTarget() {
        return inputs.closestTargetYaw;
    }

    public double getPitchToClosestTarget() {
        return inputs.closestTargetPitch;
    }

    public List<PhotonTrackedTarget> getTargets() {
        return inputs.targets;
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
        public double closestTargetYaw;
        public double closestTargetPitch;
        public List<PhotonTrackedTarget> targets;
    }
}
