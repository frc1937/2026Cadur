package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.characterization.WheelRadiusCharacterization;
import frc.lib.generic.hardware.controllers.Controller;
import frc.lib.generic.hardware.controllers.KeyboardController;
import frc.lib.util.flippable.Flippable;
import frc.robot.commands.HubShotTuning;
import frc.robot.commands.pathfinding.BLineTuner;
import frc.robot.subsystems.shooter.ShooterStates;
import frc.robot.subsystems.swerve.SwerveCommands;
import frc.robot.utilities.MatchStateTracker;
import frc.robot.utilities.PathingConstants;

import java.util.function.DoubleSupplier;

import static frc.lib.generic.hardware.controllers.Controller.Axis.LEFT_X;
import static frc.lib.generic.hardware.controllers.Controller.Axis.LEFT_Y;
import static frc.lib.generic.hardware.controllers.Controller.Stick.LEFT_STICK;
import static frc.lib.generic.hardware.controllers.Controller.Stick.RIGHT_STICK;
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
        TUNE_BLINE,
        TUNE_HUB_SHOTS,
        TUNE_INTAKE
    }


    private static final Controller DRIVER_CONTROLLER = new Controller(0);
    private static final KeyboardController OPERATOR_CONTROLLER = new KeyboardController();

    public static final DoubleSupplier DRIVE_SIGN = () -> Flippable.isRedAlliance() ? 1 : -1;

    private static final DoubleSupplier X_SUPPLIER = () -> DRIVE_SIGN.getAsDouble() * DRIVER_CONTROLLER.getRawAxis(LEFT_Y);
    private static final DoubleSupplier Y_SUPPLIER = () -> DRIVE_SIGN.getAsDouble() * DRIVER_CONTROLLER.getRawAxis(LEFT_X);
    private static final DoubleSupplier OMEGA_SUPPLIER = () -> -DRIVER_CONTROLLER.getRawAxis(Controller.Axis.RIGHT_X) * 8;

    private static final Trigger USER_BUTTON = new Trigger(RobotController::getUserButton);

    public static void initializeButtons(ButtonLayout layout) {
        switch (layout) {
            case TELEOP -> configureButtonsTeleop();
            case DEVELOPMENT -> configureButtonsDevelopment();
            case CHARACTERIZE_WHEEL_RADIUS -> configureButtonsCharacterizeWheelRadius();
            case CHARACTERIZE_SWERVE_DRIVE_MOTORS -> setupDriveMotorsCharacterization();
            case CHARACTERIZE_SWERVE_AZIMUTH -> setupAzimuthCharacterization();
            case TUNE_BLINE -> configureBLineTuning();
            case TUNE_INTAKE -> configureIntakeMechanism();
            case TUNE_HUB_SHOTS -> configureHubShooting();
        }
    }

    private static void configureHubShooting() {
        setupDriving();

        DRIVER_CONTROLLER.getStick(RIGHT_STICK)
                .whileTrue(HubShotTuning.shootFromDashboard());

        DRIVER_CONTROLLER.getButton(Controller.Inputs.RIGHT_BUMPER)
                .whileTrue(SHOOTER_STATES.setCurrentState(ShooterStates.ShooterState.SHOOTING_HUB))
                .onFalse(SHOOTER_STATES.setCurrentState(ShooterStates.ShooterState.IDLE));

        DRIVER_CONTROLLER.getStick(LEFT_STICK)
                .whileTrue(TURRET.trackHubIdly());

        DRIVER_CONTROLLER.getDPad(Controller.DPad.UP).whileTrue(INTAKE.testDeployment(-6));

        DRIVER_CONTROLLER.getButton(Controller.Inputs.A).whileTrue(HubShotTuning.confirmMake());
        DRIVER_CONTROLLER.getButton(Controller.Inputs.X).whileTrue(HubShotTuning.confirmMiss());
    }

    private static void configureIntakeMechanism() {
        DRIVER_CONTROLLER.getButton(Controller.Inputs.RIGHT_BUMPER).whileTrue(INTAKE.calibrateIntakeZero());

        setupSysIdCharacterization(INTAKE);

        DRIVER_CONTROLLER.getStick(RIGHT_STICK).whileTrue(INTAKE.testRollerDeployment(6));

        DRIVER_CONTROLLER.getDPad(Controller.DPad.UP).whileTrue(INTAKE.testDeployment(-6));
        DRIVER_CONTROLLER.getDPad(Controller.DPad.DOWN).whileTrue(INTAKE.testDeployment(6));
        DRIVER_CONTROLLER.getDPad(Controller.DPad.LEFT).whileTrue(INTAKE.testDeployment(-3));
        DRIVER_CONTROLLER.getDPad(Controller.DPad.RIGHT).whileTrue(INTAKE.testDeployment(3));
    }

    private static void configureBLineTuning() {
        setupDriving();

        final BLineTuner tuner = new BLineTuner(
                PathingConstants.BLINE_TRANSLATION_PID,
                PathingConstants.BLINE_ROTATION_PID,
                PathingConstants.BLINE_CROSS_TRACK_PID
        );//todo tune this bs lmfao

        tuner.configureController(DRIVER_CONTROLLER, new Pose2d(new Translation2d(1,2), Rotation2d.fromDegrees(60)));
    }

    private static void configureButtonsDevelopment() {
        DRIVER_CONTROLLER.getButton(Controller.Inputs.A).whileTrue(KICKER.run().alongWith(FLYWHEEL.setTarget(20)));
        DRIVER_CONTROLLER.getButton(Controller.Inputs.B).whileTrue(KICKER.run().alongWith(FLYWHEEL.setTarget(40)));
        DRIVER_CONTROLLER.getButton(Controller.Inputs.X).whileTrue(KICKER.run().alongWith(FLYWHEEL.setTarget(60)));
        DRIVER_CONTROLLER.getButton(Controller.Inputs.Y).whileTrue(KICKER.run().alongWith(FLYWHEEL.setTarget(80)));
    }

    private static void configureButtonsTeleop() {
        setupDriving();

        TURRET.setDefaultCommand(TURRET.trackHubIdly());
        HOOD.setDefaultCommand(HOOD.duckHood());

        //automatic shooting
        (IS_HUB_ACTIVE.and(IS_IN_ALLIANCE_ZONE))
                .whileTrue(SHOOTER_STATES.setCurrentState(ShooterStates.ShooterState.SHOOTING_HUB))
                .onFalse(SHOOTER_STATES.setCurrentState(ShooterStates.ShooterState.IDLE));

        //intake
        DRIVER_CONTROLLER.getStick(RIGHT_STICK)
                .toggleOnTrue(INTAKE.deployIntake().andThen(INTAKE.grabBallsUnadjusted()))
                .onFalse(INTAKE.retractIntake());

        //pass
        DRIVER_CONTROLLER.getButton(Controller.Inputs.LEFT_BUMPER)
                .toggleOnTrue(SHOOTER_STATES.setCurrentState(ShooterStates.ShooterState.SHOOTING_PASSING))
                .onFalse(SHOOTER_STATES.setCurrentState(ShooterStates.ShooterState.IDLE));

        setupOperatorKeyboardButtons();
        setupTeleopLEDs();
    }

    private static void setupOperatorKeyboardButtons() {
        // Override: Blue alliance won autonomous
        OPERATOR_CONTROLLER.seven().onTrue(Commands.runOnce(() -> MatchStateTracker.getInstance().setManualOverride(false)));
        // Ignore hub state entirely (always allow shooting)
        OPERATOR_CONTROLLER.eight().onTrue(Commands.runOnce(() -> MatchStateTracker.getInstance().toggleIgnoreHubState()));
        // Override: Red alliance won autonomous
        OPERATOR_CONTROLLER.nine().onTrue(Commands.runOnce(() -> MatchStateTracker.getInstance().setManualOverride(true)));
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
        //TODO
    }

    private static void setupDriveMotorsCharacterization() {
        setupDriving();
        setupSysIdCharacterization(SWERVE);
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
        SWERVE.setDefaultCommand(SwerveCommands.driveOpenLoopAssisted(
                        X_SUPPLIER,
                        Y_SUPPLIER,
                        OMEGA_SUPPLIER,
                        () -> false
                ));

//        DRIVER_CONTROLLER.getButton(Controller.Inputs.START).whileTrue(SwerveCommands.resetGyro());
//        DRIVER_CONTROLLER.getButton(Controller.Inputs.BACK).whileTrue(SwerveCommands.lockSwerve());
    }
}
