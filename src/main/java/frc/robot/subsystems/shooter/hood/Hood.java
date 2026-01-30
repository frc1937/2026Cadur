package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;
import org.littletonrobotics.junction.Logger;

import static frc.robot.RobotContainer.TURRET;
import static frc.robot.subsystems.shooter.hood.HoodConstants.HOOD_MECHANISM;
import static frc.robot.subsystems.shooter.hood.HoodConstants.HOOD_MOTOR;

public class Hood extends GenericSubsystem {
    public Command setHoodPosition(double position) {
        return new FunctionalCommand(
                () -> {
                },
                () -> HOOD_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, position),
                interrupt -> HOOD_MOTOR.stopMotor(),
                () -> false,
                this
        );
    }

    public Rotation2d getCurrentHoodPosition() {
        return Rotation2d.fromRotations(HOOD_MOTOR.getSystemPosition());
    }

    public Rotation2d getTargetHoodPosition() {
        return Rotation2d.fromRotations(HOOD_MOTOR.getClosedLoopTarget());
    }

    public Command stopHood() {
        return Commands.runOnce(HOOD_MOTOR::stopMotor, this);
    }

    public void printPose() {
        if (HOOD_MECHANISM != null) {
            final Pose3d current3dPose = new Pose3d(new Translation3d(0, 0, 0.75), new Rotation3d(0, getCurrentHoodPosition().getRadians(), TURRET.getCurrentTurretPosition().getRadians()));

            Logger.recordOutput("Components/HoodPose", current3dPose);

            HOOD_MECHANISM.updateCurrentAngle(getCurrentHoodPosition());
            HOOD_MECHANISM.updateTargetAngle(getTargetHoodPosition());
        }
    }
}
