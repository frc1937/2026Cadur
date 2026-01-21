package frc.robot.poseestimation.camera;

import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.Logger;

public class Camera {
    private final String prefix;
    private final Transform3d cameraToRobot;

    private final CameraIO cameraIO;
    private final CameraIOInputsAutoLogged inputs = new CameraIOInputsAutoLogged();

    public Camera(String name, Transform3d cameraToRobot, CameraIO.PoseStrategy strategy) {
        cameraIO = CameraIO.generateCamera(name, cameraToRobot, strategy);
        prefix = "Camera/" + name;

        this.cameraToRobot = cameraToRobot;
    }

    public void refreshInputs() {
        cameraIO.updateInputs(inputs);
        Logger.processInputs(prefix, inputs);
    }

    public EstimateData[] getEstimates() { return inputs.estimations; }

    public boolean cameraHasResults() {
        return inputs.hasResult;
    }

    public Transform3d getCameraToRobot() {
        return cameraToRobot;
    }
}
