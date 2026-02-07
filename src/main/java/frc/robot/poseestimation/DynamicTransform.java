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
     * @param timestamp the timestamp for the camera transform
     * @return the robot's pose on the field
     */
    public Pose2d getRobotPose(Pose2d cameraPose, double timestamp) {
        return cameraPose.transformBy(getCameraToRobot(timestamp));
    }

    /**
     * Transforms a camera pose to a robot pose in 3D.
     *
     * @param cameraPose       the camera's pose in 3D space
     * @param timestamp the timestamp for the camera transform
     * @return the robot's pose in 3D space
     */
    public Pose3d getRobotPose(Pose3d cameraPose, double timestamp) {
        return cameraPose.transformBy(get3dCameraToRobot(timestamp));
    }

    /**
     * Gets the 2D transform from camera to robot center.
     * Only x, y, and yaw components are preserved to avoid pitch and roll inaccuracies.
     *
     * @param timestamp the timestamp for the transform
     * @return the 2D transform from camera to robot center
     */
    public Transform2d getCameraToRobot(double timestamp) {
        return getRobotToCamera(timestamp).inverse();
    }

    /**
     * Gets the 2D transform from robot center to camera.
     * Only x, y, and yaw components are preserved to avoid pitch and roll inaccuracies.
     *
     * @param timestamp the timestamp for the transform
     * @return the 2D transform from robot center to camera
     */
    public Transform2d getRobotToCamera(double timestamp) {
        return toTransform2d(get3dRobotToCamera(timestamp));
    }

    /**
     * Gets the 3D transform from camera to robot center.
     *
     * @param timestamp the timestamp for the transform
     * @return the 3D transform from camera to robot center
     */
    public Transform3d get3dCameraToRobot(double timestamp) {
        return get3dRobotToCamera(timestamp).inverse();
    }

    /**
     * Gets the 3D transform from robot center to camera.
     *
     * @param timestamp the timestamp for the transform
     * @return the 3D transform from robot center to camera
     */
    public Transform3d get3dRobotToCamera(double timestamp) {
        return robotToCameraFunction.apply(timestamp);
    }
}