package frc.robot.subsystems.intake;


import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.generic.hardware.motor.*;
import frc.lib.generic.simulation.SimulationProperties;

import static frc.lib.generic.hardware.motor.MotorProperties.SparkType.MAX;
import static frc.robot.utilities.PortsConstants.IntakePorts.INTAKE_MOTOR_PORT;

public class IntakeConstants {
    protected static final Motor INTAKE_MOTOR = MotorFactory.createSpark("Intake Motor", INTAKE_MOTOR_PORT, MAX);

    static {
        configureIntakeMotor();
    }

    private static void configureIntakeMotor() {
        final MotorConfiguration intakeMotorConfiguration = new MotorConfiguration();

        intakeMotorConfiguration.inverted = true;
        intakeMotorConfiguration.simulationSlot = new MotorProperties.Slot(1, 0, 0, 0, 0, 0);
        intakeMotorConfiguration.idleMode = MotorProperties.IdleMode.COAST;
        intakeMotorConfiguration.simulationProperties = new SimulationProperties.Slot(
                SimulationProperties.SimulationType.SIMPLE_MOTOR,
                DCMotor.getFalcon500(1),
                150,
                0.02);

        INTAKE_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);
        INTAKE_MOTOR.configure(intakeMotorConfiguration);
    }
}