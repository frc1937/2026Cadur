package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.util.flippable.FlippableRotation2d;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.robot.RobotContainer.DETECTION_CAMERA;
import static frc.robot.RobotContainer.SWERVE;
import static frc.robot.subsystems.swerve.SwerveConstants.*;
import static frc.robot.subsystems.swerve.SwerveModuleConstants.MODULES;

public class SwerveCommands {
    public static Command stopDriving() {
        return new InstantCommand(SWERVE::stop);
    }

    public static Command driveToNearestTarget() {
        return Commands.run(
                () -> {
                    if (!DETECTION_CAMERA.hasResult()) return;

                    final double yawError = DETECTION_CAMERA.getYawToClosestTarget();
                    final double pitchError = 0 - DETECTION_CAMERA.getPitchToClosestTarget();

                    final double rotationSpeed = yawError * YAW_ERROR_PID_KP;
                    final double forwardSpeed = pitchError * PITCH_ERROR_PID_KP;

                    SWERVE.driveRobotRelative(new ChassisSpeeds(forwardSpeed, 0, rotationSpeed), false);
                }, //TODO test
                SWERVE
        );
    }

    public static Command lockSwerve() {
        return Commands.run(
                () -> {
                    final SwerveModuleState
                            right = new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                            left = new SwerveModuleState(0, Rotation2d.fromDegrees(45));

                    MODULES[0].setTargetState(left, false);
                    MODULES[1].setTargetState(right, false);
                    MODULES[2].setTargetState(right, false);
                    MODULES[3].setTargetState(left, false);
                },
                SWERVE
        );
    }

    public static Command goToPosePID(Pose2d targetPose) {
        return new FunctionalCommand(
                () -> {
                    Logger.recordOutput("Poses/Targets/TargetPIDPose", targetPose);

                    SWERVE.resetRotationController();
                    SWERVE.setGoalRotationController(targetPose.getRotation());
                },
                () -> SWERVE.driveToPosePID(targetPose),
                interrupt -> SWERVE.stop(),
                () -> SWERVE.isAtPose(targetPose, 0.044, 0.4),
                SWERVE
        );
    }

    public static Command resetGyro() {
        return Commands.runOnce(() -> SWERVE.setGyroHeading(Rotation2d.fromDegrees(0)), SWERVE);
    }

    public static Command driveWithTimeout(double x, double y, double rotation, boolean robotCentric, double timeout) {
        return new FunctionalCommand(
                () -> SWERVE.driveOpenLoop(x, y, rotation, robotCentric),
                () -> SWERVE.driveOpenLoop(x, y, rotation, robotCentric),
                interrupt -> {},
                () -> false,
                SWERVE
        ).withTimeout(timeout).andThen(stopDriving());
    }

    public static Command driveOpenLoop(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rotation, BooleanSupplier robotCentric) {
        return Commands.run(
                () -> SWERVE.driveOpenLoop(x.getAsDouble(), y.getAsDouble(), rotation.getAsDouble(), robotCentric.getAsBoolean()),
                SWERVE
        );
    }

    public static Command driveWhilstRotatingToTarget(DoubleSupplier x, DoubleSupplier y, Pose2d target, BooleanSupplier robotCentric) {
        return new FunctionalCommand(
                () -> {
                    SWERVE.resetRotationController();
                    SWERVE.setGoalRotationController(target.getRotation());
                },
                () -> SWERVE.driveWithTarget(x.getAsDouble(), y.getAsDouble(), robotCentric.getAsBoolean()),
                interrupt -> {},
                () -> false,
                SWERVE
        );
    }

    public static Command rotateToTarget(Pose2d target) {
        return rotateToTarget(target.getRotation());
    }

    public static Command rotateToTarget(FlippableRotation2d rotationTarget) {
        return rotateToTarget(rotationTarget.get());
    }

    public static Command rotateToTarget(Rotation2d rotationTarget) {
        return new FunctionalCommand(
                () -> {
                    SWERVE.resetRotationController();
                    SWERVE.setGoalRotationController(rotationTarget);
                },
                SWERVE::rotateToTargetFromPresetGoal,
                interrupt -> {},
                SWERVE_ROTATION_CONTROLLER::atGoal,
                SWERVE
        );
    }
}
