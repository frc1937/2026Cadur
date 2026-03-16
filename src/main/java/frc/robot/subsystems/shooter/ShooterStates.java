package frc.robot.subsystems.shooter;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static frc.robot.RobotContainer.*;
import static frc.robot.subsystems.intake.IntakeConstants.IntakeState.*;
import static java.lang.Math.hypot;

public class ShooterStates {
    public enum ShooterState {
        IDLE,
        SHOOTING_HUB,
        SHOOTING_HUB_KICKER_ACCELERATING,
        SHOOTING_PASSING,
        SHOOTING_PASSING_HUB_BLOCKED,
        NOTHING
    }

    private ShooterState state = ShooterState.IDLE;

    public Command setState(ShooterState newState) {
        return Commands.runOnce(() -> {
            state = newState;

            if (state == ShooterState.SHOOTING_PASSING || state == ShooterState.SHOOTING_HUB) {
                if (INTAKE.getState() == RETRACTED || INTAKE.getState() == DEPLOYED_NO_ROLLER) {
                    CommandScheduler.getInstance().schedule(INTAKE.setState(SHOOTING));
                }
            }
        });
    }

    @AutoLogOutput(key = "Shooter/State")
    public ShooterState getState() {
        return state;
    }

    public boolean isReadyToShoot() {
        final ChassisSpeeds v = SWERVE.getRobotRelativeVelocity();

        Logger.recordOutput("Shooter/TurretReady", TURRET.isReadyToShootPhysics());
        Logger.recordOutput("Shooter/HoodReady", HOOD.isReadyToShootPhysics());
        Logger.recordOutput("Shooter/FlywheelReady", FLYWHEEL.isReadyToShootSOTM());

        return TURRET.isReadyToShootPhysics()
                && HOOD.isReadyToShootPhysics()
                && FLYWHEEL.isReadyToShootSOTM()
                && hypot(v.vxMetersPerSecond, v.vyMetersPerSecond) <= 3;
    }
}
