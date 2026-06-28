package frc.robot.poseestimation;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.poseestimation.camera.Camera;
import frc.robot.poseestimation.camera.CameraIO;

import java.util.HashMap;
import java.util.List;

import static frc.robot.RobotContainer.TURRET;

public class PoseEstimatorConstants {
    public static final Pose2d DEFAULT_POSITION = new Pose2d(14,4.07, Rotation2d.kPi);

    public static final Matrix<N3, N1> QUEST_STD_DEVS = VecBuilder.fill(0.02, 0.02, 0.035);
    public static final Matrix<N3, N1> ODOMETRY_STD_DEVS = VecBuilder.fill(0.003, 0.003, 0.0002);

    public static final double VISION_STD_LINEAR = 0.005; //TODO Change back: 0.01, 0.09
    public static final double VISION_STD_ANGULAR = 0.09;

    public static final double VISION_FLYWHEEL_DISTRUST_FACTOR = 0.05;

    public static double MAX_Z_ERROR = 0.75;

    public static final Camera TURRET_CAMERA = new Camera(
            "FRONT_RIGHT",
            new DynamicTransform((timestamp) -> TURRET.getCameraTransform(timestamp)), //mustnt be :: else crash
            CameraIO.PoseStrategy.MULTI_TAG_COPROCESSOR
    );

    private static final List<Integer> TAGS_TO_IGNORE = List.of();

    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = createAprilTagFieldLayout();
    public static final HashMap<Integer, Pose3d> TAG_ID_TO_POSE = fieldLayoutToTagIdToPoseMap();

    private static AprilTagFieldLayout createAprilTagFieldLayout() {
        return AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    }

    private static HashMap<Integer, Pose3d> fieldLayoutToTagIdToPoseMap() {
        final HashMap<Integer, Pose3d> tagIdToPose = new HashMap<>();

        for (AprilTag aprilTag : APRIL_TAG_FIELD_LAYOUT.getTags()) {
            if (!TAGS_TO_IGNORE.contains(aprilTag.ID))
                tagIdToPose.put(aprilTag.ID, aprilTag.pose);
        }

        return tagIdToPose;
    }
}

