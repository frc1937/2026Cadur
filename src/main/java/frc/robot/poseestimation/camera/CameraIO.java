package frc.robot.poseestimation.camera;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.GlobalConstants;
import org.littletonrobotics.junction.AutoLog;

import static frc.robot.GlobalConstants.CURRENT_MODE;

public class CameraIO {
    public enum PoseStrategy {
        CONSTRAINED_PNP,
        MULTI_TAG_COPROCESSOR
    }

    public static CameraIO generateCamera(String name, Transform3d cameraToRobot, PoseStrategy strategy) {
        if (CURRENT_MODE == GlobalConstants.Mode.REPLAY)
            return new CameraIO();

        if (CURRENT_MODE == GlobalConstants.Mode.SIMULATION) {
//            return new CameraPhotonSimulation(name, cameraToRobot);
            return new CameraIO();
        }

        return new CameraPhotonReal(name, cameraToRobot, strategy);
    }

    public void updateInputs(CameraIOInputsAutoLogged inputs) {}

    @AutoLog
    public static class CameraIOInputs {
        public EstimateData[] estimations;
        public boolean hasResult;
    }
}
