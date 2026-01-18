package frc.robot.poseestimation.camera;


import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import static frc.lib.math.Optimizations.isRobotFlat;
import static frc.robot.poseestimation.PoseEstimatorConstants.*;
import static frc.robot.subsystems.swerve.SwerveConstants.GYRO;

public record EstimateData(Pose3d pose, double timestamp, double distanceFromTag, CameraIO.PoseStrategy strategy) {
    public boolean isValid() {
        final boolean invalidPose = Math.abs(pose.getZ()) > MAX_Z_ERROR
                || pose.getX() < 0.0
                || pose.getX() > APRIL_TAG_FIELD_LAYOUT.getFieldLength()
                || pose.getY() < 0.0
                || pose.getY() > APRIL_TAG_FIELD_LAYOUT.getFieldWidth();

        if (invalidPose) return false;

        if (strategy == CameraIO.PoseStrategy.CONSTRAINED_PNP) { //ROBOT SHOULD BE STRAIGHT.
            return isRobotFlat();
        }

        return true;
    }

    public Matrix<N3, N1> getStandardDeviations() {
        final double standardDeviationFactor = distanceFromTag * distanceFromTag;

        final double linearStandardDeviation = VISION_STD_LINEAR * standardDeviationFactor;
        final double angularStandardDeviation = VISION_STD_ANGULAR * standardDeviationFactor;

        return VecBuilder.fill(linearStandardDeviation, linearStandardDeviation, angularStandardDeviation);
    }
}