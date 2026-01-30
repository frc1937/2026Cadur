package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;
import static frc.lib.generic.hardware.motor.MotorProperties.ControlMode.VOLTAGE;
import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.RobotContainer.SWERVE;
import static frc.robot.subsystems.shooter.turret.TurretConstants.*;
import static frc.robot.utilities.FieldConstants.HUB_TOP_POSITION;

public class Turret extends GenericSubsystem {
    private static final double LOOKAHEAD_SECONDS = 0.045;

    public Command getMaxValues() {
        return new Command() {
            private double maxVelocity = 0;
            private double maxAcceleration = 0;

            @Override
            public void execute() {
                TURRET_MOTOR.setOutput(VOLTAGE, 12.0);

                double currentVelocity = TURRET_MOTOR.getSystemVelocity();
                double currentAcceleration = TURRET_MOTOR.getSystemAcceleration();

                if (Math.abs(currentVelocity) > Math.abs(maxVelocity))
                    maxVelocity = currentVelocity;
                if (Math.abs(currentAcceleration) > Math.abs(maxAcceleration))
                    maxAcceleration = currentAcceleration;
            }

            @Override
            public void end(boolean interrupted) {
                TURRET_MOTOR.stopMotor();

                System.out.println("--- TURRET CHARACTERIZATION RESULTS ---");
                System.out.println("Peak Velocity: " + maxVelocity + " RPS");
                System.out.println("Peak Acceleration: " + maxAcceleration + " RPS/s");
                System.out.println("Suggested kV: " + (12.0 / maxVelocity));
                System.out.println("---------------------------------------");
            }
        };
    }

    public Command homeToHUB() {
        return new FunctionalCommand(
                () -> {},
                () -> {
                    ChassisSpeeds speeds = SWERVE.getRobotRelativeVelocity();
                    Pose2d currentPose = POSE_ESTIMATOR.getCurrentPose();

                    Pose2d futurePose = currentPose.exp(
                            new Twist2d(
                                    speeds.vxMetersPerSecond * LOOKAHEAD_SECONDS,
                                    speeds.vyMetersPerSecond * LOOKAHEAD_SECONDS,
                                    speeds.omegaRadiansPerSecond * LOOKAHEAD_SECONDS
                            )
                    );

                    Rotation2d fieldRelativeAngle = HUB_TOP_POSITION.get().toTranslation2d()
                            .minus(futurePose.getTranslation())
                            .getAngle();

                    Rotation2d robotRelativeAngle = fieldRelativeAngle.minus(futurePose.getRotation());

                    double rawRotations = robotRelativeAngle.getRotations();
                    double optimizedRotations = MathUtil.inputModulus(rawRotations, MIN_ANGLE.getRotations(), MAX_ANGLE.getRotations());

                    setTargetPosition(optimizedRotations);
                },
                interrupt -> {},
                () -> false,
                this
        );
    }


    public Command stop() {
        return Commands.runOnce(TURRET_MOTOR::stopMotor, this);
    }

    public Rotation2d getCurrentPosition() {
        return Rotation2d.fromRotations(TURRET_MOTOR.getSystemPosition());
    }

    public Rotation2d getTargetPosition() {
        return Rotation2d.fromRotations(TURRET_MOTOR.getClosedLoopTarget());
    }

    public void printPose() {
        if (TURRET_MECHANISM != null) {
            final Rotation2d currentTurretPosition = getCurrentPosition();
            final Rotation2d targetTurretPosition = getTargetPosition();
            final Pose3d current3dPose = new Pose3d(0, 0, 0.5, new Rotation3d(0, 0, currentTurretPosition.getRadians()));

            Logger.recordOutput("Components/TurretPose", current3dPose);

            TURRET_MECHANISM.updateCurrentAngle(currentTurretPosition);
            TURRET_MECHANISM.updateTargetAngle(targetTurretPosition);
        }
    }

    @Override
    public SysIdRoutine.Config getSysIdConfig() {
        return SYSID_TURRET_CONFIG;
    }

    @Override
    public void sysIdDrive(double voltage) {
        TURRET_MOTOR.setOutput(VOLTAGE, voltage);
    }

    @Override
    public void sysIdUpdateLog(SysIdRoutineLog log) {
        log.motor("TURRET_MOTOR_YAW" + TURRET_MOTOR.getDeviceID())
                .voltage(Volts.of(TURRET_MOTOR.getVoltage()))
                .angularPosition(Rotations.of(TURRET_MOTOR.getSystemPosition()))
                .angularVelocity(RotationsPerSecond.of(TURRET_MOTOR.getSystemVelocity()));
    }

    /**
     * @Units in rotations
     */
    private void setTargetPosition(double targetPosition) {
        TURRET_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, targetPosition);
    }
}