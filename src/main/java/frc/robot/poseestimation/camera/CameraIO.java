package frc.robot.poseestimation.camera;

import frc.robot.GlobalConstants;
import frc.robot.poseestimation.DynamicTransform;
import org.littletonrobotics.junction.AutoLog;

import static frc.robot.GlobalConstants.CURRENT_MODE;

public class CameraIO {
    public enum PoseStrategy {
        CONSTRAINED_PNP,
        MULTI_TAG_COPROCESSOR
    }

    public static CameraIO generateCamera(String name, DynamicTransform robotToCamera, PoseStrategy strategy) {
        if (CURRENT_MODE == GlobalConstants.Mode.REPLAY)
            return new CameraIO();

        if (CURRENT_MODE == GlobalConstants.Mode.SIMULATION) {
//            return new CameraPhotonSimulation(name, robotToCamera);
            return new CameraIO();
        }

        return new CameraPhotonReal(name, robotToCamera, strategy);
    }

    public void updateInputs(CameraIOInputsAutoLogged inputs) {}

    @AutoLog
    public static class CameraIOInputs {
        public EstimateData[] estimations;
        public boolean hasResult;
    }
}
