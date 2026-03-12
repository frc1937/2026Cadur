package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import static frc.robot.RobotContainer.*;

public class HubShotTuning {
    private static final String KEY = "HubShotTuner/";

    private final static LoggedNetworkNumber hoodAngleDegrees = new LoggedNetworkNumber(KEY + "hoodAngleDeg", 14.0);
    private final static LoggedNetworkNumber flywheelSpeedRPS = new LoggedNetworkNumber(KEY + "flywheelSpeedRPS", 30.0);

    public static Command shootFromDashboard() {
        final Trigger isAtGoal = new Trigger(() -> HOOD.isAtGoal() && FLYWHEEL.isAtGoal());

        return Commands.parallel(
                Commands.run(
                        () -> {
                    Logger.recordOutput("Shooter/HoodReady", HOOD.isAtGoal());
                    Logger.recordOutput("Shooter/FlywheelReady", FLYWHEEL.isAtGoal());
                }),
                HOOD.setTarget(() -> hoodAngleDegrees.get() / 360.0),
                FLYWHEEL.setTarget(flywheelSpeedRPS::get),
                Commands.waitUntil(isAtGoal).andThen(KICKER.setAtRPS(flywheelSpeedRPS.get()).alongWith(REVOLVER.enableRevolver()))
        );
    }
}