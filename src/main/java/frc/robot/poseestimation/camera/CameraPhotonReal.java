package frc.robot.poseestimation.camera;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.poseestimation.DynamicTransform;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.ArrayList;
import java.util.Optional;

import static frc.lib.math.Optimizations.isRobotFlat;
import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.poseestimation.PoseEstimatorConstants.APRIL_TAG_FIELD_LAYOUT;
import static frc.robot.poseestimation.PoseEstimatorConstants.TAG_ID_TO_POSE;

public class CameraPhotonReal extends CameraIO {
    private final PhotonPoseEstimator poseEstimator;
    private final PoseStrategy strategy;
    private final PhotonCamera camera;

    private final DynamicTransform dynamicTransform;

    public CameraPhotonReal(String name, DynamicTransform robotToCamera, PoseStrategy strategy) {
        this.camera = new PhotonCamera(name);
        this.dynamicTransform = robotToCamera;

        this.strategy = strategy;
        this.poseEstimator = new PhotonPoseEstimator(APRIL_TAG_FIELD_LAYOUT, new Transform3d());
    }

    @Override
    public void updateInputs(CameraIOInputsAutoLogged inputs) {
        if (!camera.isConnected()) return;

        poseEstimator.addHeadingData(Timer.getFPGATimestamp(), POSE_ESTIMATOR.getCurrentAngle());

        var results = camera.getAllUnreadResults();

        if (results == null || results.isEmpty()) {
            inputs.hasResult = false;
            inputs.estimations = null;
            return;
        }

        final ArrayList<EstimateData> estimations = new ArrayList<>();

        for (PhotonPipelineResult result : results) {
            if (!result.hasTargets()) continue;

            Optional<EstimatedRobotPose> visionEstimation = poseEstimator.estimateCoprocMultiTagPose(result);

            if (visionEstimation.isEmpty())
                visionEstimation = poseEstimator.estimateLowestAmbiguityPose(result);

            if (strategy == PoseStrategy.CONSTRAINED_PNP) {
                if (visionEstimation.isEmpty()) continue;

                final boolean headingFree = DriverStation.isDisabled();

                final Optional<EstimatedRobotPose> constrainedPNPPose = poseEstimator.estimateConstrainedSolvepnpPose(
                        result,
                        getCameraMatrix(),
                        getDistCoefficient(),
                        visionEstimation.get().estimatedPose,

                        headingFree,
                        1);

                if (!isRobotFlat() && !applyToInputs(inputs, constrainedPNPPose, estimations))
                    applyToInputs(inputs, visionEstimation, estimations);
            } else if (strategy == PoseStrategy.MULTI_TAG_COPROCESSOR) {
                applyToInputs(inputs, visionEstimation, estimations);
            }

        }

        if (estimations.isEmpty()) {
            inputs.hasResult = false;
            return;
        }

        inputs.estimations = estimations.toArray(new EstimateData[0]);
    }

    private boolean applyToInputs(CameraIOInputsAutoLogged inputs, Optional<EstimatedRobotPose> visionEstimation, ArrayList<EstimateData> estimations) {
        if (visionEstimation.isEmpty()) {
            inputs.hasResult = false;
            return false;
        }

        final Pose3d tagPose = TAG_ID_TO_POSE.get(visionEstimation.get().targetsUsed.get(0).fiducialId);
        final Pose3d robotPose = dynamicTransform.getRobotPose(visionEstimation.get().estimatedPose, visionEstimation.get().timestampSeconds);

        estimations.add(new EstimateData(
                robotPose,
                visionEstimation.get().timestampSeconds,
                robotPose.getTranslation().getDistance(tagPose.getTranslation()),
                strategy));

        inputs.hasResult = true;

        return true;
    }

    private Matrix<N8, N1> getDistCoefficient() {
        return camera.getDistCoeffs().isEmpty() ? camera.getDistCoeffs().orElseThrow() : camera.getDistCoeffs().get();
    }

    private Matrix<N3, N3> getCameraMatrix() {
        return camera.getCameraMatrix().isEmpty() ? camera.getCameraMatrix().orElseThrow() : camera.getCameraMatrix().get();
    }
}
