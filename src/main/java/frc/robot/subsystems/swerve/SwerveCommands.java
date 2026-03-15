package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.util.flippable.FlippableRotation2d;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.math.MathUtil.*;
import static edu.wpi.first.math.geometry.Rotation2d.fromRadians;
import static edu.wpi.first.wpilibj2.command.Commands.run;
import static frc.robot.RobotContainer.*;
import static frc.robot.subsystems.swerve.SwerveConstants.SWERVE_ROTATION_CONTROLLER;
import static frc.robot.subsystems.swerve.SwerveConstants.TRENCH_CORRECTION_Y_CONTROLLER;
import static frc.robot.subsystems.swerve.SwerveModuleConstants.MODULES;
import static frc.robot.utilities.FieldConstants.Trench.getClosestTrenchToRobot;
import static java.lang.Math.abs;

public class SwerveCommands {
    public static Command stopDriving() {
        return new InstantCommand(SWERVE::stop);
    }

    public static Command lockSwerve() {
        return run(
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
                    SWERVE.setTargetRotation(targetPose.getRotation());
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

    public static Command driveOpenLoop(DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega, BooleanSupplier robotCentric) {
        return run(
                () -> SWERVE.driveOpenLoop(x.getAsDouble(), y.getAsDouble(), omega.getAsDouble(), robotCentric.getAsBoolean()),
                SWERVE
        );
    }

    public static Command driveWhilstRotatingToTarget(DoubleSupplier x, DoubleSupplier y, Pose2d target, BooleanSupplier robotCentric) {
        return new FunctionalCommand(
                () -> {
                    SWERVE.resetRotationController();
                    SWERVE.setTargetRotation(target.getRotation());
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
                    SWERVE.setTargetRotation(rotationTarget);
                },
                SWERVE::rotateToTargetFromPresetGoal,
                interrupt -> {},
                SWERVE_ROTATION_CONTROLLER::atGoal,
                SWERVE
        );
    }


    public static Command driveOpenLoopAssisted(DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega, BooleanSupplier snakeMode) {
        return run(
                () -> {
                    final double xValue = x.getAsDouble();
                    final double yValue = y.getAsDouble();
                    final double omegaValue = omega.getAsDouble();

                    if (IS_IN_TRENCH_AREA.getAsBoolean()) {
                        final double current = POSE_ESTIMATOR.getPose().getY();
                        final double target = getClosestTrenchToRobot(POSE_ESTIMATOR.getPose()).get().getY();
                        final double error = abs(current - target);

                        double trenchCorrectionValue = TRENCH_CORRECTION_Y_CONTROLLER.calculate(current, target);

                        if (TRENCH_CORRECTION_Y_CONTROLLER.atSetpoint())
                            trenchCorrectionValue = 0;

                        final double yAssistAmount = clamp(error * 2.0, 0, 0.9);
                        final double omegaAssistAmount = clamp(abs(getClosestStraightAngle() - SWERVE.getGyroHeading()) * 2.0, 0, 0.9);

                        SWERVE.driveOpenLoop(
                                xValue,
                                interpolate(yValue, trenchCorrectionValue, yAssistAmount),
                                interpolate(omegaValue, SWERVE.getOmegaToTarget(getClosestStraightAngle()), omegaAssistAmount),
                                false);
                        return;
                    }

                    if (snakeMode.getAsBoolean() && (xValue != 0 || yValue != 0)) {
                        final Rotation2d targetAngle = fromRadians(Math.atan2(yValue, xValue));
                        SWERVE.driveOpenLoop(xValue, yValue, SWERVE.getOmegaToTarget(targetAngle.getRotations()), false);
                    } else
                        SWERVE.driveOpenLoop(xValue, yValue, omegaValue, false);
                },
                SWERVE
        );
    }

    private static double getClosestStraightAngle() {
        final double computedAngle = inputModulus(SWERVE.getGyroHeading(), -0.5, 0.5);
        final double difference = (computedAngle - 0.25);

        return (difference < 0.25 || difference > -0.25) ? 0 : 0.5;
    }
}
