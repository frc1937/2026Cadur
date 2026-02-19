package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.revrobotics.util.StatusLogger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.generic.hardware.HardwareManager;
import frc.robot.subsystems.leds.Leds;
import frc.robot.utilities.MatchStateTracker;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

import static frc.robot.RobotContainer.*;
import static frc.robot.utilities.PathingConstants.initializeBLine;

public class Robot extends LoggedRobot {
    private final CommandScheduler commandScheduler = CommandScheduler.getInstance();
    private RobotContainer robotContainer;

    @Override
    public void robotInit() {
        SignalLogger.enableAutoLogging(false);
        StatusLogger.disableAutoLogging();
      
        initializeBLine();
        robotContainer = new RobotContainer();
        HardwareManager.initialize(this);
    }

    @Override
    public void robotPeriodic() {
        HardwareManager.update();
        commandScheduler.run();

        Logger.recordOutput("isRedHubActive:", MatchStateTracker.isHubActive());
        POSE_ESTIMATOR.periodic();

        SHOOTING_CALCULATOR.clearLatestParameters();
    }

    @Override
    public void disabledPeriodic() {
        LEDS.setToDefault();
    }

    @Override
    public void autonomousInit() {
        final Command autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null)
            commandScheduler.schedule(autonomousCommand);

        LEDS.setLEDStatus(Leds.LEDMode.AUTO_START,2).andThen(LEDS.setLEDStatus(Leds.LEDMode.AUTOMATION,0));
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void simulationPeriodic() {
        HardwareManager.updateSimulation();

        robotContainer.updateComponentPoses();
    }
}