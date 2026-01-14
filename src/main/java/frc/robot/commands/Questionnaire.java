package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Questionnaire {
    private final LoggedDashboardChooser<String> PRESET_QUESTION;
    private final Cycle
            CYCLE_1,
            CYCLE_2,
            CYCLE_3;

    public Questionnaire() {
        PRESET_QUESTION = createPresetQuestion();

        CYCLE_1 = initializeCycleFromKey("1");
        CYCLE_2 = initializeCycleFromKey("2");
        CYCLE_3 = initializeCycleFromKey("3");
    }

    private Cycle initializeCycleFromKey(String key) {
        return new Cycle(
                createExampleQuestion(key)
        );
    }

    private LoggedDashboardChooser<String> createPresetQuestion() {
        final LoggedDashboardChooser<String> question = new LoggedDashboardChooser<>("Which Auto?");

        return question;
    }

    private LoggedDashboardChooser<Command> createExampleQuestion(String cycleNumber) {
        final LoggedDashboardChooser<Command> question = new LoggedDashboardChooser<>(cycleNumber + " Example?");

        question.addDefaultOption("Example 1", Commands.none());
        question.addOption("Example 2", Commands.none());

        return question;
    }


    private Command createCycleSequence(Cycle cycle) {
        return new Command() {};
    }

    public Command getCommand() {
        return Commands.sequence(
                createCycleSequence(CYCLE_1),
                createCycleSequence(CYCLE_2),
                createCycleSequence(CYCLE_3)
        );
    }

    public String getSelected() {
        return PRESET_QUESTION.getSendableChooser().getSelected() != "None" ? PRESET_QUESTION.get() : "Custom";
    }

    private record Cycle(
            LoggedDashboardChooser<Command> ExampleQuestion) {
    }
}