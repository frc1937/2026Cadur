package frc.robot.poseestimation.camera;


import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import static frc.lib.math.Optimizations.isRobotFlat;
import static frc.robot.poseestimation.PoseEstimatorConstants.*;
import static frc.robot.utilities.FieldConstants.FIELD_LENGTH;
import static frc.robot.utilities.FieldConstants.FIELD_WIDTH;

public record EstimateData(Pose3d pose, double timestamp, double distanceFromTag, int tagCount, CameraIO.PoseStrategy strategy) {
    public boolean isValid() {
        final boolean invalidPose = Math.abs(pose.getZ()) > MAX_Z_ERROR
                || pose.getX() < 0.0
                || pose.getX() > FIELD_LENGTH
                || pose.getY() < 0.0
                || pose.getY() > FIELD_WIDTH;

        return !invalidPose;
    }

    public Matrix<N3, N1> getStandardDeviations() {
        final double standardDeviationFactor = distanceFromTag * distanceFromTag;
        final double tagCountFactor = 1.0 / Math.sqrt(tagCount);

        final double linearStandardDeviation = VISION_STD_LINEAR * standardDeviationFactor * tagCountFactor;
        final double angularStandardDeviation = VISION_STD_ANGULAR * standardDeviationFactor * tagCountFactor;

        return VecBuilder.fill(linearStandardDeviation, linearStandardDeviation, angularStandardDeviation);
    }
}