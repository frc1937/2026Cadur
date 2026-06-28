package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.lib.util.flippable.FlippablePose2d;
import frc.robot.lib.BLine.Path;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import static edu.wpi.first.math.geometry.Rotation2d.kPi;
import static edu.wpi.first.math.geometry.Rotation2d.kZero;
import static frc.lib.util.flippable.Flippable.isRedAlliance;
import static frc.robot.RobotContainer.*;
import static frc.robot.commands.pathfinding.PathfindingCommands.pathfindAndFollow;
import static frc.robot.subsystems.intake.IntakeConstants.IntakeState.DEPLOYED;
import static frc.robot.subsystems.shooter.ShooterStates.ShooterState.IDLE;
import static frc.robot.subsystems.shooter.ShooterStates.ShooterState.SHOOTING_HUB;
import static frc.robot.utilities.FieldConstants.*;

public class Questionnaire {
    private static final Transform2d TRENCH_TO_ROBOT_START = new Transform2d(new Translation2d(-0.4, 0), kZero);

    private final LoggedDashboardChooser<StartingPose> CHOOSE_STARTING_POSE;
    private final LoggedDashboardChooser<CollectionPose> CHOOSE_ALLIANCE_COLLECTION;
    private final LoggedDashboardChooser<SecondCycle> CHOOSE_SECOND_CYCLE;

    private enum StartingPose {
        TRENCH_TOP(new FlippablePose2d(BOTTOM_TRENCH.mirroredY().getMiddle(), kZero, false, true),
                BALLS_TOP_START, 1),
        TRENCH_BOTTOM(new FlippablePose2d(BOTTOM_TRENCH.getMiddle(), kZero, false, true),
                BALLS_BOTTOM_START, -1);

        private final FlippablePose2d startingPose;
        private final FlippablePose2d beginIntakingPose;
        private final double sign;

        StartingPose(FlippablePose2d startingPose, FlippablePose2d beginIntakingPose, double sign) {
            this.startingPose = startingPose;
            this.beginIntakingPose = beginIntakingPose;
            this.sign = sign;
        }

        public Pose2d getPose() {
            return startingPose.get().transformBy(TRENCH_TO_ROBOT_START);
        }

        public Pose2d getBeginIntakingPose() {
            return beginIntakingPose.get();
        }

        public double getSign() {
            return sign;
        }
    }

    private enum CollectionPose {
        DEPOT(DEPOT_LOCATION),
        OUTPOST(OUTPOST_LOCATION),
        SHOOT_BOTTOM(new FlippablePose2d(new Translation2d(2.604766, HALF_FIELD_WIDTH - 2), kZero, false, true)),
        SHOOT_TOP(new FlippablePose2d(new Translation2d(2.604766, HALF_FIELD_WIDTH + 2), kZero, false, true));

        private final FlippablePose2d startingPose;

        CollectionPose(FlippablePose2d startingPose) {
            this.startingPose = startingPose;
        }

        public Pose2d getPose() {
            return startingPose.get();
        }
    }

    private enum SecondCycle {
        YES(),
        NO();

        SecondCycle() {
        }

        public boolean getValue() {
            return this == YES;
        }
    }

    public Questionnaire() {
        CHOOSE_STARTING_POSE = createQuestion("Which trench side?", StartingPose.class);
        CHOOSE_ALLIANCE_COLLECTION = createQuestion("Where to collect from?", CollectionPose.class);
        CHOOSE_SECOND_CYCLE = createQuestion("Do a 254-style second cycle?", SecondCycle.class);
    }

    public Command getCommand() {
        final StartingPose start = CHOOSE_STARTING_POSE.get();
        final CollectionPose collect = CHOOSE_ALLIANCE_COLLECTION.get();
        final SecondCycle cycle = CHOOSE_SECOND_CYCLE.get();

        if (start == null || collect == null || cycle == null) return null;

        final Pose2d middleOfField = start.getBeginIntakingPose().transformBy(new Transform2d(-2.5,0,kZero));
        final Pose2d middleOfHub = middleOfField.transformBy(
                new Transform2d(0, 2 * (isRedAlliance() ? (-start.getSign()) : (start.getSign())), kPi));

        final Pose2d shiftedIntakingPose = start.getBeginIntakingPose().transformBy(
            new Transform2d(0, isRedAlliance() ? (-start.getSign()) : (start.getSign()), kZero));

        final Path.PathConstraints slowDriveConstraints = new Path.PathConstraints().setMaxVelocityMetersPerSec(1.2);
        final Path.PathConstraints mediumDriveConstraints = new Path.PathConstraints().setMaxVelocityMetersPerSec(2.5);

        final Command intakeAndFollowPath = pathfindAndFollow(middleOfField, slowDriveConstraints).alongWith(INTAKE.setState(DEPLOYED));

        final ConditionalCommand secondCycleCommand = new ConditionalCommand(
                (pathfindAndFollow(start.startingPose.get()).alongWith(INTAKE.setState(DEPLOYED)))
                        .andThen(pathfindAndFollow(shiftedIntakingPose, 2))
                        .andThen(pathfindAndFollow(middleOfHub, mediumDriveConstraints))
                        .andThen(pathfindAndFollow(collect.getPose(), mediumDriveConstraints)),
                Commands.idle(),
                cycle::getValue);

        return (SHOOTER_STATES.setState(IDLE).alongWith(INTAKE.setState(DEPLOYED)).alongWith(pathfindAndFollow(start.getPose())))
                .andThen(pathfindAndFollow(start.getBeginIntakingPose()))
                .andThen(intakeAndFollowPath)
                .andThen(pathfindAndFollow(collect.getPose()).alongWith(SHOOTER_STATES.setState(SHOOTING_HUB).onlyWhile(IS_IN_ALLIANCE_ZONE)))
                .andThen(Commands.waitSeconds(1))
                .andThen(secondCycleCommand);
    }

    public String getSelected() {
        final StartingPose start = CHOOSE_STARTING_POSE.get();
        final CollectionPose collect = CHOOSE_ALLIANCE_COLLECTION.get();
        final SecondCycle cycle = CHOOSE_SECOND_CYCLE.get();

        return (start == null || collect == null) ? "Custom" : start.name() + " + " + collect.name() + " + Second-Cycle: " + cycle.name();
    }

    private <T extends Enum<T>> LoggedDashboardChooser<T> createQuestion(String questionName, Class<T> enumClass) {
        final LoggedDashboardChooser<T> question = new LoggedDashboardChooser<>(questionName);

        for (final T option : enumClass.getEnumConstants()) {
            question.addOption(option.name(), option);
        }

        if (enumClass.getEnumConstants().length > 0) {
            question.addDefaultOption(enumClass.getEnumConstants()[0].name(), enumClass.getEnumConstants()[0]);
        }

        return question;
    }
}