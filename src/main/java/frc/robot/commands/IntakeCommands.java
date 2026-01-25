package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveCommands;

import static frc.robot.RobotContainer.INTAKE;

public class IntakeCommands {
    public static Command intakeClosestBall() {
        return INTAKE.enableIntakeAdjusted().alongWith(SwerveCommands.driveToNearestTarget());
    }
}
