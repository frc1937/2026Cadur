package frc.robot.subsystems.intake;


import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.generic.hardware.motor.*;
import frc.lib.generic.simulation.SimulationProperties;

import static frc.lib.generic.hardware.motor.MotorProperties.SparkType.FLEX;
import static frc.robot.utilities.PortsConstants.IntakePorts.*;

public class IntakeConstants {
    protected static final Motor INTAKE_GRAB_MOTOR = MotorFactory.createSpark("INTAKE_GRAB_MOTOR", INTAKE_GRAB_MOTOR_PORT, FLEX);
    protected static final Motor INTAKE_EXTENSION_MOTOR = MotorFactory.createSpark("INTAKE_EXTENSION_MOTOR", INTAKE_EXTENSION_MOTOR_PORT, FLEX);

    static final double MINIMUM_INTAKE_SPEED_TANGENTIAL_MPS = 3;
    static final double INTAKE_WHEEL_DIAMETER_METERS = 0.04; //TODO TUNE

    static final double INTAKE_RETRACTED_POSITION = 0;
    static final double INTAKE_DEPLOYED_POSITION = 0.5;

    static {
        configureIntakeGrabMotor();
        configureIntakeExtensionMotor();
    }

    private static void configureIntakeExtensionMotor() {
        final MotorConfiguration config = new MotorConfiguration();

        config.idleMode = MotorProperties.IdleMode.BRAKE;
        config.gearRatio = 1; //todo tu ne

        config.slot = new MotorProperties.Slot(10, 0, 0, 0, 0, 0);//todo: TUNE position control, sysid

        config.forwardSoftLimit = INTAKE_DEPLOYED_POSITION;
        config.reverseSoftLimit = INTAKE_RETRACTED_POSITION;

        config.simulationSlot = new MotorProperties.Slot(1, 0, 0, 0, 0, 0);
        config.simulationProperties = new SimulationProperties.Slot(
                SimulationProperties.SimulationType.SIMPLE_MOTOR,
                DCMotor.getFalcon500(1),
                1,
                0.2);

        INTAKE_GRAB_MOTOR.configure(config);

        INTAKE_GRAB_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);
        INTAKE_GRAB_MOTOR.setupSignalUpdates(MotorSignal.POSITION);
        INTAKE_GRAB_MOTOR.setupSignalUpdates(MotorSignal.VELOCITY);
        INTAKE_GRAB_MOTOR.setupSignalUpdates(MotorSignal.CLOSED_LOOP_TARGET);
    }

    private static void configureIntakeGrabMotor() {
        final MotorConfiguration config = new MotorConfiguration();

        config.idleMode = MotorProperties.IdleMode.COAST;
        config.inverted = true;

        config.slot = new MotorProperties.Slot(10, 0, 0, 0, 0, 0);
        //todo: TUNE velocity controller, sysid

        config.simulationSlot = new MotorProperties.Slot(1, 0, 0, 0, 0, 0);
        config.simulationProperties = new SimulationProperties.Slot(
                SimulationProperties.SimulationType.SIMPLE_MOTOR,
                DCMotor.getFalcon500(1),
                1,
                0.2);

        INTAKE_GRAB_MOTOR.configure(config);

        INTAKE_GRAB_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);
        INTAKE_GRAB_MOTOR.setupSignalUpdates(MotorSignal.VELOCITY);
        INTAKE_GRAB_MOTOR.setupSignalUpdates(MotorSignal.CLOSED_LOOP_TARGET);
    }
}