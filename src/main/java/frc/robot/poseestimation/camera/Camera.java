package frc.robot.poseestimation.camera;

import frc.robot.poseestimation.DynamicTransform;
import org.littletonrobotics.junction.Logger;

public class Camera {
    private final String prefix;

    private final CameraIO cameraIO;
    private final CameraIOInputsAutoLogged inputs = new CameraIOInputsAutoLogged();

    public Camera(String name, DynamicTransform robotToCamera, CameraIO.PoseStrategy strategy) {
        cameraIO = CameraIO.generateCamera(name, robotToCamera, strategy);
        prefix = "Camera/" + name;
    }

    public void refreshInputs() {
        cameraIO.updateInputs(inputs);
        Logger.processInputs(prefix, inputs);
    }

    public EstimateData[] getEstimates() { return inputs.estimations; }

    public boolean cameraHasResults() {
        return inputs.hasResult;
    }
}
