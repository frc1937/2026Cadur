package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.characterization.WheelRadiusCharacterization;
import frc.lib.generic.hardware.controllers.Controller;
import frc.lib.generic.hardware.controllers.KeyboardController;
import frc.lib.generic.hardware.motor.MotorProperties;
import frc.lib.util.flippable.Flippable;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.shooter.ShooterStates;
import frc.robot.subsystems.swerve.SwerveCommands;
import frc.robot.utilities.MatchStateTracker;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import static frc.lib.generic.hardware.controllers.Controller.Axis.LEFT_X;
import static frc.lib.generic.hardware.controllers.Controller.Axis.LEFT_Y;
import static frc.robot.RobotContainer.*;
import static frc.robot.subsystems.swerve.SwerveCommands.rotateToTarget;
import static frc.robot.utilities.PathingConstants.ROBOT_CONFIG;

public class ButtonControls {
    public enum ButtonLayout {
        DEVELOPMENT,
        TELEOP,
        CHARACTERIZE_SWERVE_DRIVE_MOTORS,
        CHARACTERIZE_WHEEL_RADIUS,
        CHARACTERIZE_SWERVE_AZIMUTH,
        TUNING
    }

    private static final Controller DRIVER_CONTROLLER = new Controller(0);
    private static final KeyboardController OPERATOR_CONTROLLER = new KeyboardController();

    public static final DoubleSupplier DRIVE_SIGN = () -> Flippable.isRedAlliance() ? 1 : -1;

    private static final DoubleSupplier X_SUPPLIER = () -> DRIVE_SIGN.getAsDouble() * DRIVER_CONTROLLER.getRawAxis(LEFT_Y);
    private static final DoubleSupplier Y_SUPPLIER = () -> DRIVE_SIGN.getAsDouble() * DRIVER_CONTROLLER.getRawAxis(LEFT_X);
    private static final DoubleSupplier ROTATION_SUPPLIER = () -> -DRIVER_CONTROLLER.getRawAxis(Controller.Axis.RIGHT_X) * 8;

    private static final Trigger USER_BUTTON = new Trigger(RobotController::getUserButton);

    public static void initializeButtons(ButtonLayout layout) {
        setupUserButtonDebugging();

        switch (layout) {
            case TELEOP -> configureButtonsTeleop();
            case DEVELOPMENT -> configureButtonsDevelopment();
            case CHARACTERIZE_WHEEL_RADIUS -> configureButtonsCharacterizeWheelRadius();
            case CHARACTERIZE_SWERVE_DRIVE_MOTORS -> {
                setupDriving();
                setupSysIdCharacterization(SWERVE);
            }
            case CHARACTERIZE_SWERVE_AZIMUTH -> setupAzimuthCharacterization();
            case TUNING -> configureButtonsForTuning();
        }
    }

    private static void configureButtonsForTuning() {
//        EasyTuner easyTuner = new EasyTuner(SwerveModuleConstants.FL_STEER_MOTOR, SWERVE, DRIVER_CONTROLLER, MotorProperties.ControlMode.POSITION);

//        easyTuner.configureController();
    }

    private static void configureButtonsDevelopment() {
        setupDriving();

        DRIVER_CONTROLLER.getButton(Controller.Inputs.A).whileTrue(KICKER.run()
                .alongWith(FLYWHEEL.setTarget(30)));

        DRIVER_CONTROLLER.getButton(Controller.Inputs.B).whileTrue(KICKER.run()
                .alongWith(FLYWHEEL.setTarget(50)));

        DRIVER_CONTROLLER.getButton(Controller.Inputs.X).whileTrue(KICKER.run()
                .alongWith(FLYWHEEL.setTarget(70)));

        DRIVER_CONTROLLER.getButton(Controller.Inputs.Y).whileTrue(KICKER.run()
                .alongWith(FLYWHEEL.setTarget(90)));

//        DRIVER_CONTROLLER.getButton(Controller.Inputs.A).whileTrue();
//        DRIVER_CONTROLLER.getButton(Controller.Inputs.B).whileTrue();
//        DRIVER_CONTROLLER.getButton(Controller.Inputs.X).whileTrue();
//        DRIVER_CONTROLLER.getButton(Controller.Inputs.Y).whileTrue();
    }

    private static void configureButtonsTeleop() {
        setupDriving();


        DRIVER_CONTROLLER.getButton(Controller.Inputs.LEFT_BUMPER).toggleOnTrue(SHOOTER_STATES.setCurrentState(ShooterStates.ShooterState.SHOOTING_HUB));


        setupOperatorKeyboardButtons();
        setupTeleopLEDs();
    }

    private static void setupOperatorKeyboardButtons() {
        OPERATOR_CONTROLLER.seven().onTrue(Commands.runOnce(() -> MatchStateTracker.setManualOverride(false)));
        OPERATOR_CONTROLLER.eight().onTrue(Commands.runOnce(() -> MatchStateTracker.setIgnoreHubState(true)));
        OPERATOR_CONTROLLER.nine().onTrue(Commands.runOnce(() -> MatchStateTracker.setManualOverride(true)));
    }


    private static void configureButtonsCharacterizeWheelRadius() {
        setupDriving();

        final Command wheelRadiusCharacterization = new WheelRadiusCharacterization(
                SWERVE,
                ROBOT_CONFIG.moduleLocations,
                SWERVE::getDriveWheelPositionsRadians,
                () -> SWERVE.getGyroHeading() * 2 * Math.PI,
                (speed) -> SWERVE.driveRobotRelative(new ChassisSpeeds(0, 0, speed), true)
        );

        DRIVER_CONTROLLER.getButton(Controller.Inputs.A).whileTrue((wheelRadiusCharacterization));
    }

    private static void setupSysIdCharacterization(GenericSubsystem subsystem) {
        DRIVER_CONTROLLER.getButton(Controller.Inputs.A).whileTrue(subsystem.getSysIdQuastatic(SysIdRoutine.Direction.kForward));
        DRIVER_CONTROLLER.getButton(Controller.Inputs.B).whileTrue(subsystem.getSysIdQuastatic(SysIdRoutine.Direction.kReverse));
        DRIVER_CONTROLLER.getButton(Controller.Inputs.Y).whileTrue(subsystem.getSysIdDynamic(SysIdRoutine.Direction.kForward));
        DRIVER_CONTROLLER.getButton(Controller.Inputs.X).whileTrue(subsystem.getSysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    private static void setupTeleopLEDs() {
        final Trigger isEndOfMatch = new Trigger(() -> DriverStation.getMatchTime() <= 30);

        isEndOfMatch.onTrue(LEDS.setLEDStatus(Leds.LEDMode.END_OF_MATCH, 5));
    }

    private static void setupUserButtonDebugging() {
        USER_BUTTON.toggleOnTrue(
                Commands.startEnd(
                                () -> setModeOfAllSubsystems(MotorProperties.IdleMode.COAST),
                                () -> setModeOfAllSubsystems(MotorProperties.IdleMode.BRAKE)
                        ).alongWith(LEDS.setLEDStatus(Leds.LEDMode.DEBUG_MODE, 1500))
                        .andThen(LEDS.setLEDStatus(Leds.LEDMode.DEFAULT, 0))
        ).debounce(1);
    }

    private static void setModeOfAllSubsystems(MotorProperties.IdleMode idleMode) {
    }

    private static void setupAzimuthCharacterization() {
        DRIVER_CONTROLLER.getButton(Controller.Inputs.A).whileTrue(
                rotateToTarget(POSE_ESTIMATOR.getPose().rotateBy(Rotation2d.fromDegrees(90))));

        DRIVER_CONTROLLER.getButton(Controller.Inputs.B).whileTrue(
                rotateToTarget(POSE_ESTIMATOR.getPose().rotateBy(Rotation2d.fromDegrees(180))));

        DRIVER_CONTROLLER.getButton(Controller.Inputs.X).whileTrue(
                rotateToTarget(POSE_ESTIMATOR.getPose().rotateBy(Rotation2d.fromDegrees(270))));

        DRIVER_CONTROLLER.getButton(Controller.Inputs.Y).whileTrue(
                rotateToTarget(POSE_ESTIMATOR.getPose().rotateBy(Rotation2d.fromDegrees(360))));
    }

    private static void setupDriving() {
        SWERVE.setDefaultCommand(
                SwerveCommands.driveOpenLoop(
                        X_SUPPLIER,
                        Y_SUPPLIER,
                        ROTATION_SUPPLIER,

                        () -> false
                ));

        DRIVER_CONTROLLER.getButton(Controller.Inputs.START).whileTrue(SwerveCommands.resetGyro());
        DRIVER_CONTROLLER.getButton(Controller.Inputs.BACK).whileTrue(SwerveCommands.lockSwerve());
    }
}
