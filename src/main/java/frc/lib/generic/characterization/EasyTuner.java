package frc.lib.generic.characterization;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.controllers.Controller;
import frc.lib.generic.hardware.motor.Motor;
import frc.lib.generic.hardware.motor.MotorConfiguration;
import frc.lib.generic.hardware.motor.MotorProperties;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class EasyTuner {
    private final Motor motor;
    private final Controller controller;
    private final MotorProperties.ControlMode mode;
    private final GenericSubsystem subsystem;

    private final LoggedNetworkNumber kP;
    private final LoggedNetworkNumber kI;
    private final LoggedNetworkNumber kD;

    private final LoggedNetworkNumber kS;
    private final LoggedNetworkNumber kV;
    private final LoggedNetworkNumber kA;

    private final LoggedNetworkNumber target = new LoggedNetworkNumber("EasyTuner/target", 0);

    public EasyTuner(Motor motor, GenericSubsystem subsystem, Controller controller, MotorProperties.ControlMode controlMode) {
        this.motor = motor;
        this.subsystem = subsystem;
        this.controller = controller;
        this.mode = controlMode;

        kP = new LoggedNetworkNumber("EasyTuner/kP", motor.getConfig().slot.kP());
        kI = new LoggedNetworkNumber("EasyTuner/kI", motor.getConfig().slot.kI());
        kD = new LoggedNetworkNumber("EasyTuner/kD", motor.getConfig().slot.kD());

        kS = new LoggedNetworkNumber("EasyTuner/kS", motor.getConfig().slot.kS());
        kV = new LoggedNetworkNumber("EasyTuner/kV", motor.getConfig().slot.kV());
        kA = new LoggedNetworkNumber("EasyTuner/kA", motor.getConfig().slot.kA());
    }

    public void configureController() {
        controller.getButton(Controller.Inputs.A).whileTrue(new FindMaxSpeedCommand(motor, subsystem));
        controller.getButton(Controller.Inputs.B).whileTrue(new StaticFrictionCharacterization(subsystem, motor, false));

        controller.getButton(Controller.Inputs.START).onTrue(new InstantCommand(this::refreshPID));

        controller.getButton(Controller.Inputs.RIGHT_BUMPER).whileTrue(runMotorToTarget());
    }

    public Command runMotorToTarget() {
        return new FunctionalCommand(
                () -> {},
                () -> motor.setOutput(mode, target.get()),
                interrupted -> motor.stopMotor(),
                () -> false,
                subsystem
        );
    }

    public void refreshPID() {
        final MotorConfiguration config = motor.getConfig();

        config.slot = new MotorProperties.Slot(kP.get(), kI.get(), kD.get(), kV.get(),kA.get(), kS.get());

        motor.configure(config);
        System.out.println("Refreshed PID with values: " + motor.getConfig().slot);
    }
}
