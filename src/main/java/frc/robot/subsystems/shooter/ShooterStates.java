package frc.robot.subsystems.shooter;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.*;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static frc.robot.RobotContainer.*;
import static java.lang.Math.hypot;

public class ShooterStates {
    public enum ShooterState {
        IDLE,
        SHOOTING_HUB,
        SHOOTING_PASSING,
        NOTHING
    }

    private ShooterState state = ShooterState.IDLE;

    public Command setState(ShooterState newState) {
        return Commands.runOnce(() -> {
            state = newState;
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
        Logger.recordOutput("Shooter/FlywheelReady", FLYWHEEL.isReadyToShootPhysics());

        return TURRET.isReadyToShootPhysics()
                && HOOD.isReadyToShootPhysics()
                && FLYWHEEL.isReadyToShootPhysics()
                && hypot(v.vxMetersPerSecond, v.vyMetersPerSecond) <= 5.0;
    }
}
