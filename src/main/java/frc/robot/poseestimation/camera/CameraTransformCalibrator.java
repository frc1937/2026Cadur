package frc.robot.poseestimation.camera;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.poseestimation.PoseEstimatorConstants;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

import static frc.robot.RobotContainer.POSE_ESTIMATOR;

/**
 * Automatically calibrates a camera's Transform3d relative to a robot or a rotating mechanism
 * (e.g., a turret). Run during disabled mode while moving the robot near AprilTags.
 *
 * <h2>How to use:</h2>
 * <ol>
 *   <li><b>Pitch:</b> Read from PhotonVision GUI – place an AprilTag directly in front of the
 *       camera at the same height and note the pitch offset shown in the GUI.</li>
 *   <li><b>Z:</b> Measure manually with a ruler (camera height above the robot/turret center).</li>
 *   <li>Deploy code, disable the robot, and drive/rotate it in front of AprilTags while
 *       {@link #setEnabled(boolean)} is true.</li>
 *   <li>After {@link #hasResult()} returns true, read the result from SmartDashboard key
 *       {@code CameraCalibration/JavaString} and paste it into your constants file.</li>
 * </ol>
 *
 * <h2>Algorithm summary:</h2>
 * <ul>
 *   <li><b>Yaw:</b> At each observation the gyro-based bearing to the tag in mechanism frame is
 *       compared to the camera-measured bearing. Their difference is the camera yaw offset.
 *       The turret angle (if provided) is subtracted to give the offset relative to the turret
 *       center rather than the robot.</li>
 *   <li><b>X, Y:</b> With the averaged camera yaw and known pitch, the tag translation in camera
 *       frame is rotated into mechanism frame. Subtracting that from the known tag position in
 *       mechanism frame yields the camera's (X, Y) offset.</li>
 *   <li><b>Pitch / Roll / Z:</b> Supplied externally (pitch from PhotonVision GUI, roll = 0,
 *       Z measured manually).</li>
 * </ul>
 */
public class CameraTransformCalibrator {

    // Minimum samples before a result is published.
    private static final int MIN_OBSERVATIONS = 30;

    // Seconds between consecutive samples (avoids correlated noise when robot is stationary).
    private static final double SAMPLE_INTERVAL_SECONDS = 0.1;

    // Skip observations where the tag is too far away (noisy at long range).
    private static final double MAX_TAG_DISTANCE_METERS = 4.0;

    // Reject a new yaw sample if it deviates more than this from the running mean.
    private static final double OUTLIER_YAW_THRESHOLD_RADIANS = Math.toRadians(15);

    // Reject a new X/Y sample if its distance from the running mean exceeds this.
    private static final double OUTLIER_XY_THRESHOLD_METERS = 0.5;

    private final PhotonCamera camera;

    /** Camera pitch relative to horizontal, in radians. Positive = tilted upward. */
    private final double cameraPitchRadians;

    /** Camera height above the mechanism origin (robot center or turret center), in metres. */
    private final double cameraZ;

    /**
     * Returns the mechanism's yaw angle in radians at the moment of a sample.
     * For a robot-fixed camera supply {@code () -> 0.0}.
     * For a turret-mounted camera supply {@code () -> TURRET.getSelfRelativePosition().getRadians()}.
     */
    private final DoubleSupplier mechanismYawRadiansSupplier;

    private final List<Double> yawSamples = new ArrayList<>();
    private final List<Double> xSamples   = new ArrayList<>();
    private final List<Double> ySamples   = new ArrayList<>();

    private double lastSampleTime = 0;
    private Transform3d computedTransform = null;
    private boolean enabled = false;

    // ─── Constructors ─────────────────────────────────────────────────────────

    /**
     * Creates a calibrator for a <b>robot-fixed</b> camera.
     *
     * @param cameraName         PhotonVision camera name (must match the name in the PhotonVision GUI)
     * @param cameraPitchDegrees Camera pitch from PhotonVision GUI (positive = tilted up)
     * @param cameraZ            Camera height above robot centre in metres (measure with a ruler)
     */
    public CameraTransformCalibrator(String cameraName, double cameraPitchDegrees, double cameraZ) {
        this(cameraName, cameraPitchDegrees, cameraZ, () -> 0.0);
    }

    /**
     * Creates a calibrator for a <b>mechanism-mounted</b> camera (e.g., turret).
     *
     * @param cameraName                     PhotonVision camera name
     * @param cameraPitchDegrees             Camera pitch from PhotonVision GUI (positive = tilted up)
     * @param cameraZ                        Camera height above the mechanism origin in metres
     * @param mechanismYawRadiansSupplier    Supplier for the mechanism's yaw angle in radians.
     *                                       For the turret use:
     *                                       {@code () -> TURRET.getSelfRelativePosition().getRadians()}
     */
    public CameraTransformCalibrator(String cameraName,
                                     double cameraPitchDegrees,
                                     double cameraZ,
                                     DoubleSupplier mechanismYawRadiansSupplier) {
        this.camera = new PhotonCamera(cameraName);
        this.cameraPitchRadians = Math.toRadians(cameraPitchDegrees);
        this.cameraZ = cameraZ;
        this.mechanismYawRadiansSupplier = mechanismYawRadiansSupplier;
    }

    // ─── Public API ───────────────────────────────────────────────────────────

    /** Enable or disable sample collection. Typically called from {@code disabledPeriodic}. */
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    /** @return true when enough observations have been collected to produce a reliable estimate */
    public boolean hasResult() {
        return computedTransform != null && yawSamples.size() >= MIN_OBSERVATIONS;
    }

    /**
     * @return the best-estimate robotToCamera (or mechanismToCamera) Transform3d,
     *         or {@code null} if not yet available
     */
    public Transform3d getComputedTransform() {
        return computedTransform;
    }

    /** Resets all samples and clears the computed result. */
    public void reset() {
        yawSamples.clear();
        xSamples.clear();
        ySamples.clear();
        computedTransform = null;
    }

    /**
     * Call this every robot loop (e.g., from {@code disabledPeriodic}).
     * Polls the camera, collects observations, and keeps the published result up to date.
     */
    public void update() {
        if (!enabled) return;
        if (Timer.getFPGATimestamp() - lastSampleTime < SAMPLE_INTERVAL_SECONDS) return;
        lastSampleTime = Timer.getFPGATimestamp();

        if (!camera.isConnected()) return;

        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        if (results == null || results.isEmpty()) return;

        for (PhotonPipelineResult result : results) {
            if (!result.hasTargets()) continue;

            PhotonTrackedTarget target = result.getBestTarget();

            Pose3d tagFieldPose = PoseEstimatorConstants.TAG_ID_TO_POSE.get(target.getFiducialId());
            if (tagFieldPose == null) continue;

            Transform3d cameraToTag = target.getBestCameraToTarget();
            if (cameraToTag.getTranslation().getNorm() > MAX_TAG_DISTANCE_METERS) continue;

            processObservation(
                    POSE_ESTIMATOR.getPose(),
                    tagFieldPose,
                    cameraToTag,
                    mechanismYawRadiansSupplier.getAsDouble()
            );
        }

        if (yawSamples.size() >= MIN_OBSERVATIONS) {
            recomputeAndLog();
        }
    }

    // ─── Core algorithm ───────────────────────────────────────────────────────

    /**
     * Extracts one yaw and one (X, Y) estimate from a single tag observation and
     * appends them to the running lists (after outlier rejection).
     *
     * <p>Coordinate conventions (WPILib / FRC):
     * <ul>
     *   <li>Robot frame: X = forward, Y = left, Z = up</li>
     *   <li>Camera frame (PhotonVision): X = forward (depth), Y = left, Z = up</li>
     *   <li>Mechanism frame = robot frame rotated by {@code -mechanismYaw}
     *       (turret zero aligns with robot forward)</li>
     * </ul>
     *
     * @param robotPose          Robot pose from odometry/pose estimator
     * @param tagFieldPose       Known AprilTag pose from the field layout
     * @param cameraToTag        PhotonVision's camera-to-tag transform
     * @param mechanismYawRad    Yaw of the mechanism (turret) in radians; 0 for robot-fixed cameras
     */
    private void processObservation(Pose2d robotPose,
                                    Pose3d tagFieldPose,
                                    Transform3d cameraToTag,
                                    double mechanismYawRad) {
        double robotHeading = robotPose.getRotation().getRadians();

        // ── Step 1: tag position in robot frame ──────────────────────────────
        double dxField = tagFieldPose.getX() - robotPose.getX();
        double dyField = tagFieldPose.getY() - robotPose.getY();

        // Rotate from field frame into robot frame (rotate by -robotHeading)
        double dxRobot = dxField * Math.cos(-robotHeading) - dyField * Math.sin(-robotHeading);
        double dyRobot = dxField * Math.sin(-robotHeading) + dyField * Math.cos(-robotHeading);

        double bearingInRobotFrame = Math.atan2(dyRobot, dxRobot);

        // ── Step 2: tag position in mechanism frame ───────────────────────────
        // Rotate robot-frame vector by -mechanismYaw to get mechanism-frame vector
        double dxMech = dxRobot * Math.cos(-mechanismYawRad) - dyRobot * Math.sin(-mechanismYawRad);
        double dyMech = dxRobot * Math.sin(-mechanismYawRad) + dyRobot * Math.cos(-mechanismYawRad);

        // ── Step 3: estimate camera yaw ───────────────────────────────────────
        // bearingInRobotFrame = mechanismYaw + cameraYaw + bearingInCameraFrame
        // => cameraYaw = bearingInRobotFrame - mechanismYaw - bearingInCameraFrame
        double bearingInCameraFrame = Math.atan2(cameraToTag.getY(), cameraToTag.getX());
        double cameraYawEstimate = normalizeAngle(
                bearingInRobotFrame - mechanismYawRad - bearingInCameraFrame
        );

        // Yaw outlier rejection
        if (!yawSamples.isEmpty()) {
            double currentMeanYaw = circularMean(yawSamples);
            if (Math.abs(normalizeAngle(cameraYawEstimate - currentMeanYaw)) > OUTLIER_YAW_THRESHOLD_RADIANS)
                return;
        }

        yawSamples.add(cameraYawEstimate);

        // ── Step 4: estimate camera X, Y ─────────────────────────────────────
        // With camera rotation (roll=0, pitch, yaw), rotate tag translation from
        // camera frame into mechanism frame:
        //   tag_in_mechanism = cameraOffset + CameraRotation * cameraToTag.translation
        // => cameraOffset = tag_in_mechanism - CameraRotation * cameraToTag.translation
        double avgYaw = circularMean(yawSamples);
        Rotation3d cameraRotation = new Rotation3d(0, cameraPitchRadians, avgYaw);
        Translation3d tagInMechFromCam = cameraToTag.getTranslation().rotateBy(cameraRotation);

        double cameraXEstimate = dxMech - tagInMechFromCam.getX();
        double cameraYEstimate = dyMech - tagInMechFromCam.getY();

        // X/Y outlier rejection
        if (!xSamples.isEmpty()) {
            double meanX = xSamples.stream().mapToDouble(d -> d).average().orElse(0);
            double meanY = ySamples.stream().mapToDouble(d -> d).average().orElse(0);
            if (Math.hypot(cameraXEstimate - meanX, cameraYEstimate - meanY) > OUTLIER_XY_THRESHOLD_METERS)
                return;
        }

        xSamples.add(cameraXEstimate);
        ySamples.add(cameraYEstimate);
    }

    // ─── Result publishing ────────────────────────────────────────────────────

    private void recomputeAndLog() {
        double yaw = circularMean(yawSamples);
        double x   = xSamples.stream().mapToDouble(d -> d).average().orElse(0);
        double y   = ySamples.stream().mapToDouble(d -> d).average().orElse(0);

        computedTransform = new Transform3d(
                new Translation3d(x, y, cameraZ),
                new Rotation3d(0, cameraPitchRadians, yaw)
        );

        // Ready-to-paste Java string for constants file
        String javaString = String.format(
                "new Transform3d(new Translation3d(%.4f, %.4f, %.4f), new Rotation3d(0, Math.toRadians(%.2f), Math.toRadians(%.2f)))",
                x, y, cameraZ,
                Math.toDegrees(cameraPitchRadians),
                Math.toDegrees(yaw)
        );

        int n = yawSamples.size();

        // SmartDashboard (readable on any dashboard)
        SmartDashboard.putNumber("CameraCalibration/X_meters",   x);
        SmartDashboard.putNumber("CameraCalibration/Y_meters",   y);
        SmartDashboard.putNumber("CameraCalibration/Z_meters",   cameraZ);
        SmartDashboard.putNumber("CameraCalibration/Pitch_deg",  Math.toDegrees(cameraPitchRadians));
        SmartDashboard.putNumber("CameraCalibration/Yaw_deg",    Math.toDegrees(yaw));
        SmartDashboard.putNumber("CameraCalibration/SampleCount", n);
        SmartDashboard.putString("CameraCalibration/JavaString", javaString);

        // AdvantageKit structured logging
        Logger.recordOutput("CameraCalibration/ComputedTransform", computedTransform);
        Logger.recordOutput("CameraCalibration/X",           x);
        Logger.recordOutput("CameraCalibration/Y",           y);
        Logger.recordOutput("CameraCalibration/Yaw_deg",     Math.toDegrees(yaw));
        Logger.recordOutput("CameraCalibration/SampleCount", n);
        Logger.recordOutput("CameraCalibration/JavaString",  javaString);

        // Driver Station console – visible during field calibration without a dashboard
        DriverStation.reportWarning(
                "[CameraCalibration] n=" + n + " | " + javaString,
                false
        );
    }

    // ─── Math helpers ─────────────────────────────────────────────────────────

    /** Wraps an angle to [-π, π]. */
    private static double normalizeAngle(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }

    /**
     * Circular (angular) mean – handles wraparound correctly so averaging angles
     * near ±π does not produce a mean near 0.
     */
    private static double circularMean(List<Double> angles) {
        double sinSum = 0, cosSum = 0;
        for (double a : angles) {
            sinSum += Math.sin(a);
            cosSum += Math.cos(a);
        }
        return Math.atan2(sinSum, cosSum);
    }
}
