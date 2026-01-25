package frc.robot.subsystems.turret.hood;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;
import org.littletonrobotics.junction.Logger;

import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.RobotContainer.TURRET;
import static frc.robot.subsystems.turret.hood.ArmConstants.ARM_MECHANISM;
import static frc.robot.subsystems.turret.hood.ArmConstants.ARM_MOTOR;
import static frc.robot.utilities.FieldConstants.HUB_POSITION;

public class Arm extends GenericSubsystem {
    public Command setArmPosition(double position) {
        return new FunctionalCommand(
                () -> {
                },
                () -> ARM_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, position),
                interrupt -> ARM_MOTOR.stopMotor(),
                () -> false,
                this
        );
    }

    public Command autoAimArm() {
        return Commands.run(() -> ARM_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, autoAim()), this);
    }

    public Rotation2d getCurrentArmPosition() {
        return Rotation2d.fromRotations(ARM_MOTOR.getSystemPosition());
    }

    public Rotation2d getTargetArmPosition() {
        return Rotation2d.fromRotations(ARM_MOTOR.getClosedLoopTarget());
    }

    public Command stopArm() {
        return Commands.runOnce(ARM_MOTOR::stopMotor, this);
    }

    public void printPose() {
        if (ARM_MECHANISM != null) {
            final Pose3d current3dPose = new Pose3d(new Translation3d(0, 0, 0.75), new Rotation3d(0, getCurrentArmPosition().getRadians(), TURRET.getCurrentTurretPosition().getRadians()));

            Logger.recordOutput("Components/ArmPose", current3dPose);

            ARM_MECHANISM.updateCurrentAngle(getCurrentArmPosition());
            ARM_MECHANISM.updateTargetAngle(getTargetArmPosition());
        }
    }

    private double autoAim() {
        final Pose2d pose = POSE_ESTIMATOR.getCurrentPose();
        final double distanceToHub = HUB_POSITION.get().getX() - pose.getX();

        return Units.radiansToRotations(Math.atan2(distanceToHub, HUB_POSITION.get().getZ())) - pose.getRotation().getRotations();
    }
}
