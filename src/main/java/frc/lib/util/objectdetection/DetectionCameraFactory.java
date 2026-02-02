package frc.lib.util.objectdetection;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.GlobalConstants;

import static frc.robot.GlobalConstants.CURRENT_MODE;

public class DetectionCameraFactory {
    public static DetectionCamera createDetectionCamera(String name, Transform3d robotToCamera) {
        if (CURRENT_MODE == GlobalConstants.Mode.REAL) {
            return new PhotonDetectionCamera(name);
        }

        if (CURRENT_MODE == GlobalConstants.Mode.REPLAY) {
            return new DetectionCamera(name);
        }

        return new SimulatedDetectionCamera(name, robotToCamera);
    }
}
