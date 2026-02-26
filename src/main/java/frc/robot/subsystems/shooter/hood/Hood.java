package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;
import frc.lib.generic.characterization.FindMaxSpeedCommand;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;
import static frc.lib.generic.hardware.motor.MotorProperties.ControlMode.VOLTAGE;
import static frc.robot.RobotContainer.SHOOTING_CALCULATOR;
import static frc.robot.RobotContainer.TURRET;
import static frc.robot.subsystems.shooter.hood.HoodConstants.*;


public class Hood extends GenericSubsystem {
    public Command trackHub() {
        return run(
                () -> {
                    final Rotation2d targetAngle = SHOOTING_CALCULATOR.getResults().hoodAngle();

                    final double constrainedTarget = MathUtil.clamp(
                            targetAngle.getRotations(),
                            MIN_ANGLE.getRotations(),
                            MAX_ANGLE.getRotations()
                    );

                    setTargetPosition(constrainedTarget);
                });
    }

    public Command getMaxValues() {
        return new FindMaxSpeedCommand(HOOD_MOTOR, this);
    }

    public Rotation2d getCurrentPosition() {
        return Rotation2d.fromRotations(HOOD_MOTOR.getSystemPosition());
    }

    public Rotation2d getTargetPosition() {
        return Rotation2d.fromRotations(HOOD_MOTOR.getClosedLoopTarget());
    }

    public Command stopHood() {
        return Commands.runOnce(HOOD_MOTOR::stopMotor, this);
    }

    public boolean isAtGoal() {
        return HOOD_MOTOR.isAtPositionSetpoint();
    }

    public void printPose() {
        if (HOOD_MECHANISM != null) {
            final Pose3d current3dPose = new Pose3d(new Translation3d(0, 0, 0.5), new Rotation3d(0, Rotation2d.fromDegrees(90).minus(getCurrentPosition()).getRadians(), TURRET.getSelfRelativePosition().getRadians()));

            Logger.recordOutput("Components/HoodPose", current3dPose);

            HOOD_MECHANISM.updateCurrentAngle(getCurrentPosition());
            HOOD_MECHANISM.updateTargetAngle(getTargetPosition());
        }
    }

    @Override
    public SysIdRoutine.Config getSysIdConfig() {
        return SYSID_HOOD_CONFIG;
    }

    @Override
    public void sysIdDrive(double voltage) {
        HOOD_MOTOR.setOutput(VOLTAGE, voltage);
    }

    @Override
    public void sysIdUpdateLog(SysIdRoutineLog log) {
        log.motor("HOOD_MOTOR" + HOOD_MOTOR.getDeviceID())
                .voltage(Volts.of(HOOD_MOTOR.getVoltage()))
                .angularPosition(Rotations.of(HOOD_MOTOR.getSystemPosition()))
                .angularVelocity(RotationsPerSecond.of(HOOD_MOTOR.getSystemVelocity()));
    }

    private void setTargetPosition(double targetPosition) {
        HOOD_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, targetPosition);
    }
}
