package frc.robot.subsystems.intake;


import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.generic.hardware.motor.*;
import frc.lib.generic.simulation.SimProperties;

import static frc.lib.generic.hardware.motor.MotorProperties.SparkType.MAX;
import static frc.robot.utilities.PortsConstants.IntakePorts.INTAKE_MOTOR_PORT;

public class IntakeConstants {
    protected static final Motor INTAKE_MOTOR = MotorFactory.createSpark("Intake Motor", INTAKE_MOTOR_PORT, MAX);

    static final double MINIMUM_INTAKE_SPEED_TANGENTIAL_MPS = 3;
    static final double INTAKE_WHEEL_DIAMETER_METERS = 0.04; //TODO TUNE

    static {
        configureIntakeMotor();
    }

    private static void configureIntakeMotor() {
        final MotorConfiguration intakeMotorConfiguration = new MotorConfiguration();

        intakeMotorConfiguration.idleMode = MotorProperties.IdleMode.COAST;
        intakeMotorConfiguration.inverted = true;

        intakeMotorConfiguration.slot = new MotorProperties.Slot(10, 0, 0, 0, 0, 0);
        //todo: TUNE velocity controller, sysid

        intakeMotorConfiguration.simulationSlot = new MotorProperties.Slot(1, 0, 0, 0, 0, 0);
        intakeMotorConfiguration.simulationProperties = new SimProperties.Slot(
                SimProperties.SimulationType.SIMPLE_MOTOR,
                DCMotor.getFalcon500(1),
                1,
                0.2);

        INTAKE_MOTOR.configure(intakeMotorConfiguration);

        INTAKE_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);
        INTAKE_MOTOR.setupSignalUpdates(MotorSignal.VELOCITY);
        INTAKE_MOTOR.setupSignalUpdates(MotorSignal.CLOSED_LOOP_TARGET);
    }
}