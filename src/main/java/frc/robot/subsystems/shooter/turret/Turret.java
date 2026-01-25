package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.shooter.turret.TurretConstants.TURRET_MECHANISM;
import static frc.robot.subsystems.shooter.turret.TurretConstants.TURRET_MOTOR;

public class Turret extends GenericSubsystem {
    public Command stop() {
        return Commands.runOnce(TURRET_MOTOR::stopMotor, this);
    }

    public Rotation2d getCurrentTurretPosition() {
        return Rotation2d.fromRotations(TURRET_MOTOR.getSystemPosition());
    }

    public Rotation2d getTargetTurretPosition() {
        return Rotation2d.fromRotations(TURRET_MOTOR.getClosedLoopTarget());
    }

    public void printPose() {
        if (TURRET_MECHANISM != null) {
            final Rotation2d currentTurretPosition = getCurrentTurretPosition();
            final Rotation2d targetTurretPosition = getTargetTurretPosition();
            final Pose3d current3dPose = new Pose3d(0, 0, 0.8, new Rotation3d(0, 0, currentTurretPosition.getRadians()));

            Logger.recordOutput("Components/TurretPose", current3dPose);

            TURRET_MECHANISM.updateCurrentAngle(currentTurretPosition);
            TURRET_MECHANISM.updateTargetAngle(targetTurretPosition);
        }
    }

    /**
     * @Units in rotations
     */
    private void setTargetPosition(double targetPosition) {
        TURRET_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, targetPosition);
    }
}