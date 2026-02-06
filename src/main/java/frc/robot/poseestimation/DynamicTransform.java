package frc.robot.poseestimation;

import edu.wpi.first.math.geometry.*;

import java.util.function.Function;

import static frc.lib.math.Conversions.toTransform2d;

/**
 * Handles transformations between camera and robot coordinate frames.
 * Supports both static and time-dependent camera positions, useful for cameras on moving mechanisms.
 */
public class DynamicTransform {
    private final Function<Double, Transform3d> robotToCameraFunction;

    /**
     * Constructs a DynamicCameraTransform with a static camera position.
     *
     * @param robotToCamera the fixed transform from robot center to camera
     */
    public DynamicTransform(Transform3d robotToCamera) {
        this(nil -> robotToCamera);
    }

    /**
     * Constructs a DynamicCameraTransform with a time-dependent camera position.
     *
     * @param robotToCameraFunction function that returns the transform from robot center to camera at a given timestamp
     */
    public DynamicTransform(Function<Double, Transform3d> robotToCameraFunction) {
        this.robotToCameraFunction = robotToCameraFunction;
    }

    /**
     * Transforms a camera pose to a robot pose in 2D.
     *
     * @param cameraPose       the camera's pose on the field
     * @param timestampSeconds the timestamp for the camera transform
     * @return the robot's pose on the field
     */
    public Pose2d getRobotPose(Pose2d cameraPose, double timestampSeconds) {
        final Transform2d cameraToRobot = getCameraToRobot(timestampSeconds);
        return cameraPose.transformBy(cameraToRobot);
    }

    /**
     * Transforms a camera pose to a robot pose in 3D.
     *
     * @param cameraPose       the camera's pose in 3D space
     * @param timestampSeconds the timestamp for the camera transform
     * @return the robot's pose in 3D space
     */
    public Pose3d getRobotPose(Pose3d cameraPose, double timestampSeconds) {
        final Transform3d cameraToRobot = get3dCameraToRobot(timestampSeconds);

        return cameraPose.transformBy(cameraToRobot);
    }

    /**
     * Gets the 2D transform from camera to robot center.
     * Only x, y, and yaw components are preserved to avoid pitch and roll inaccuracies.
     *
     * @param timestampSeconds the timestamp for the transform
     * @return the 2D transform from camera to robot center
     */
    public Transform2d getCameraToRobot(double timestampSeconds) {
        return getRobotToCamera(timestampSeconds).inverse();
    }

    /**
     * Gets the 2D transform from robot center to camera.
     * Only x, y, and yaw components are preserved to avoid pitch and roll inaccuracies.
     *
     * @param timestampSeconds the timestamp for the transform
     * @return the 2D transform from robot center to camera
     */
    public Transform2d getRobotToCamera(double timestampSeconds) {
        return toTransform2d(get3dRobotToCamera(timestampSeconds));
    }

    /**
     * Gets the 3D transform from camera to robot center.
     *
     * @param timestampSeconds the timestamp for the transform
     * @return the 3D transform from camera to robot center
     */
    public Transform3d get3dCameraToRobot(double timestampSeconds) {
        return get3dRobotToCamera(timestampSeconds).inverse();
    }

    /**
     * Gets the 3D transform from robot center to camera.
     *
     * @param timestampSeconds the timestamp for the transform
     * @return the 3D transform from robot center to camera
     */
    public Transform3d get3dRobotToCamera(double timestampSeconds) {
        return robotToCameraFunction.apply(timestampSeconds);
    }
}