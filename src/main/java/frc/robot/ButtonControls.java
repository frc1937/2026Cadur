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
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.swerve.SwerveCommands;
import frc.robot.utilities.PathingConstants;

import java.util.Set;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static frc.lib.generic.hardware.controllers.Controller.Axis.LEFT_X;
import static frc.lib.generic.hardware.controllers.Controller.Axis.LEFT_Y;
import static frc.lib.generic.hardware.controllers.Controller.Inputs.*;
import static frc.lib.generic.hardware.controllers.Controller.Stick.LEFT_STICK;
import static frc.lib.generic.hardware.controllers.Controller.Stick.RIGHT_STICK;
import static frc.lib.generic.visualization.DrawUtils.TWO_PI;
import static frc.robot.RobotContainer.*;
import static frc.robot.subsystems.intake.IntakeConstants.IntakeState.*;
import static frc.robot.subsystems.shooter.ShooterStates.ShooterState.*;
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
        TUNE_INTAKE,
        SYSTEM_TEST
    }


    private static final Controller DRIVER_CONTROLLER = new Controller(0);
    private static final KeyboardController OPERATOR_CONTROLLER = new KeyboardController();

    public static final DoubleSupplier DRIVE_SIGN = () -> Flippable.isRedAlliance() ? 1 : -1;

    private static final DoubleSupplier X_SUPPLIER = () -> DRIVE_SIGN.getAsDouble() * DRIVER_CONTROLLER.getRawAxis(LEFT_Y);
    private static final DoubleSupplier Y_SUPPLIER = () -> DRIVE_SIGN.getAsDouble() * DRIVER_CONTROLLER.getRawAxis(LEFT_X);
    private static final DoubleSupplier OMEGA_SUPPLIER = () -> -DRIVER_CONTROLLER.getRawAxis(Controller.Axis.RIGHT_X) * 10;

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
            case SYSTEM_TEST -> testSystems();
        }
    }

    private static void testSystems() {
        setupDriving();

        INTAKE.setDefaultCommand(INTAKE.followState());

        DRIVER_CONTROLLER.getButton(Controller.Inputs.RIGHT_BUMPER).whileTrue(INTAKE.calibrateIntakeZero());
        DRIVER_CONTROLLER.getButton(LEFT_BUMPER).whileTrue(HOOD.calibrateHoodZero());

        DRIVER_CONTROLLER.getButton(A).whileTrue(INTAKE.setState(DEPLOYED));
        DRIVER_CONTROLLER.getButton(B).whileTrue(INTAKE.setState(RETRACTED));

        DRIVER_CONTROLLER.getButton(X).whileTrue(HOOD.setTarget(() -> Rotation2d.fromDegrees(20).getRotations()));
        DRIVER_CONTROLLER.getButton(Y).whileTrue(HOOD.setTarget(() -> Rotation2d.fromDegrees(40).getRotations()));

        DRIVER_CONTROLLER.getDPad(Controller.DPad.RIGHT).whileTrue(TURRET.testTurret(0.5,0));
        DRIVER_CONTROLLER.getDPad(Controller.DPad.LEFT).whileTrue(TURRET.testTurret(-0.5,0));

        DRIVER_CONTROLLER.getStick(RIGHT_STICK).whileTrue(REVOLVER.enableRevolver().alongWith(KICKER.run()));
    }


    private static void configureHubShooting() {
        setupDriving();

        HOOD.setDefaultCommand(HOOD.followState());
        TURRET.setDefaultCommand(TURRET.followState());
        KICKER.setDefaultCommand(KICKER.followState());
        FLYWHEEL.setDefaultCommand(FLYWHEEL.followState());
        REVOLVER.setDefaultCommand(REVOLVER.followState());
        INTAKE.setDefaultCommand(INTAKE.followState());

        DRIVER_CONTROLLER.getStick(RIGHT_STICK).whileTrue(HubShotTuning.shootFromDashboard());
        DRIVER_CONTROLLER.getStick(LEFT_STICK).onTrue(SHOOTER_STATES.setState(IDLE));

        DRIVER_CONTROLLER.getButton(RIGHT_BUMPER).onTrue(SHOOTER_STATES.setState(SHOOTING_HUB));
        DRIVER_CONTROLLER.getButton(LEFT_BUMPER).onTrue(SHOOTER_STATES.setState(SHOOTING_PASSING));

        DRIVER_CONTROLLER.getDPad(Controller.DPad.LEFT).onTrue(INTAKE.setState(DEPLOYED));
        DRIVER_CONTROLLER.getDPad(Controller.DPad.DOWN).onTrue(INTAKE.setState(DEPLOYED_NO_ROLLER));
        DRIVER_CONTROLLER.getDPad(Controller.DPad.RIGHT).onTrue(INTAKE.setState(RETRACTED));

        DRIVER_CONTROLLER.getButton(A).whileTrue(HOOD.calibrateHoodZero());
        DRIVER_CONTROLLER.getButton(B).whileTrue(INTAKE.calibrateIntakeZero());
        DRIVER_CONTROLLER.getButton(Y).whileTrue(SHOOTER_STATES.setState(NOTHING));
        DRIVER_CONTROLLER.getButton(X).whileTrue(HOOD.calibrateHoodZero().alongWith(INTAKE.calibrateIntakeZero()));
    }

    private static void configureIntakeMechanism() {
        setupDriving();

        INTAKE.setDefaultCommand(INTAKE.followState());

        DRIVER_CONTROLLER.getButton(Controller.Inputs.RIGHT_BUMPER).whileTrue(INTAKE.calibrateIntakeZero());

        DRIVER_CONTROLLER.getButton(Controller.Inputs.A).onTrue(INTAKE.setState(DEPLOYED));
        DRIVER_CONTROLLER.getButton(Controller.Inputs.B).onTrue(INTAKE.setState(DEPLOYED_NO_ROLLER));
        DRIVER_CONTROLLER.getButton(Controller.Inputs.X).onTrue(INTAKE.setState(RETRACTED));
    }

    private static void configureBLineTuning() {
        setupDriving();

        final BLineTuner tuner = new BLineTuner(
                PathingConstants.BLINE_TRANSLATION_PID,
                PathingConstants.BLINE_ROTATION_PID,
                PathingConstants.BLINE_CROSS_TRACK_PID
        );

        tuner.configureController(DRIVER_CONTROLLER);
    }

    private static void configureButtonsDevelopment() {
        DRIVER_CONTROLLER.getButton(Controller.Inputs.A).whileTrue(KICKER.run().alongWith(FLYWHEEL.setTarget(20)));
        DRIVER_CONTROLLER.getButton(Controller.Inputs.B).whileTrue(KICKER.run().alongWith(FLYWHEEL.setTarget(40)));
        DRIVER_CONTROLLER.getButton(Controller.Inputs.X).whileTrue(KICKER.run().alongWith(FLYWHEEL.setTarget(60)));
        DRIVER_CONTROLLER.getButton(Controller.Inputs.Y).whileTrue(KICKER.run().alongWith(FLYWHEEL.setTarget(80)));
    }

    private static void configureButtonsTeleop() {
        setupDriving();

        HOOD.setDefaultCommand(HOOD.followState());
        TURRET.setDefaultCommand(TURRET.followState());
        KICKER.setDefaultCommand(KICKER.followState());
        FLYWHEEL.setDefaultCommand(FLYWHEEL.followState());
        REVOLVER.setDefaultCommand(REVOLVER.followState());

        INTAKE.setDefaultCommand(INTAKE.followState());

        //automatic shooting
        (IS_HUB_ACTIVE.and(IS_IN_ALLIANCE_ZONE))
                .onTrue(SHOOTER_STATES.setState(SHOOTING_HUB))
                .onFalse(SHOOTER_STATES.setState(IDLE));

        //intaking
        DRIVER_CONTROLLER.getButton(RIGHT_BUMPER).onTrue(Commands.defer(() ->
                INTAKE.setState(INTAKE.getState() == RETRACTED ? DEPLOYED : RETRACTED), Set.of(INTAKE)));

        DRIVER_CONTROLLER.getStick(RIGHT_STICK).onTrue(Commands.defer(() ->
                INTAKE.setState(INTAKE.getState() == DEPLOYED ? DEPLOYED_NO_ROLLER : DEPLOYED), Set.of(INTAKE)));

        //passing type shi
        DRIVER_CONTROLLER.getButton(LEFT_BUMPER).onTrue(Commands.defer(() ->
                SHOOTER_STATES.setState(SHOOTER_STATES.getState() == IDLE ? SHOOTING_PASSING : IDLE), Set.of()));

        setupOperatorKeyboardButtons();
        setupTeleopLEDs();
    }

    private static void setupOperatorKeyboardButtons() {
        // Override: Blue alliance won autonomous
        OPERATOR_CONTROLLER.seven().onTrue(runOnce(() -> MATCH_TRACKER.setManualOverride(false)));
        // Ignore hub state entirely (always allow shooting)
        OPERATOR_CONTROLLER.eight().onTrue(runOnce(() -> MATCH_TRACKER.toggleIgnoreHubState()));
        // Override: Red alliance won autonomous
        OPERATOR_CONTROLLER.nine().onTrue(runOnce(() -> MATCH_TRACKER.setManualOverride(true)));
    }


    private static void configureButtonsCharacterizeWheelRadius() {
        setupDriving();

        final Command wheelRadiusCharacterization = new WheelRadiusCharacterization(
                SWERVE,
                ROBOT_CONFIG.moduleLocations,
                SWERVE::getDriveWheelPositionsRadians,
                () -> SWERVE.getGyroHeading() * TWO_PI,
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
        // Intake
        new Trigger(() -> INTAKE.getState() == DEPLOYED).whileTrue(LEDS.show(Leds.LEDMode.INTAKE_DEPLOYED));
        new Trigger(() -> INTAKE.getState() == DEPLOYED_NO_ROLLER).whileTrue(LEDS.show(Leds.LEDMode.INTAKE_DEPLOYED_NO_ROLLER));

        new Trigger(() -> FLYWHEEL.isReadyToShootPhysics() && TURRET.isReadyToShootPhysics()).whileTrue(LEDS.show(Leds.LEDMode.READY_TO_SHOOT));
        new Trigger(() -> SHOOTER_STATES.getState() == SHOOTING_HUB).whileTrue(LEDS.show(Leds.LEDMode.SHOOTING_HUB));
        new Trigger(() -> SHOOTER_STATES.getState() == SHOOTING_PASSING).whileTrue(LEDS.show(Leds.LEDMode.PASSING));

        // Hub / match state
        IS_HUB_ACTIVE.and(IS_IN_ALLIANCE_ZONE.negate()).whileTrue(LEDS.show(Leds.LEDMode.HUB_ACTIVE_NOT_IN_ZONE));

        new Trigger(() -> MATCH_TRACKER.getCompensatedShiftTimeRemaining() < 3.0 && MATCH_TRACKER.isHubActive()).whileTrue(LEDS.show(Leds.LEDMode.SHIFT_ENDING));

        // Operator alerts
        new Trigger(MATCH_TRACKER::shouldIgnoreHubState).whileTrue(LEDS.show(Leds.LEDMode.OVERRIDE_ACTIVE));
        new Trigger(() -> !MATCH_TRACKER.isGameDataReceived()).whileTrue(LEDS.show(Leds.LEDMode.NO_GAME_DATA));
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
        final AtomicBoolean shouldSnakeMode = new AtomicBoolean(false);

        DRIVER_CONTROLLER.getDPad(Controller.DPad.UP).onTrue(runOnce(() -> shouldSnakeMode.set(!shouldSnakeMode.get())));

        SWERVE.setDefaultCommand(SwerveCommands.driveOpenLoopAssisted(
                X_SUPPLIER,
                Y_SUPPLIER,
                OMEGA_SUPPLIER,
                shouldSnakeMode::get
        ));

        DRIVER_CONTROLLER.getButton(Controller.Inputs.START).whileTrue(SwerveCommands.resetGyro());
//        DRIVER_CONTROLLER.getButton(Controller.Inputs.BACK).whileTrue(SwerveCommands.lockSwerve());
    }
}
