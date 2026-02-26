package frc.robot.subsystems.intake;


import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.generic.hardware.motor.*;
import frc.lib.generic.simulation.SimulationProperties;

import static frc.lib.generic.hardware.motor.MotorProperties.SparkType.FLEX;
import static frc.robot.utilities.PortsConstants.IntakePorts.*;

public class IntakeConstants {
    protected static final Motor INTAKE_ROLLER_MOTOR = MotorFactory.createSpark("INTAKE_ROLLER_MOTOR", INTAKE_ROLLER_MOTOR_PORT, FLEX);
    protected static final Motor INTAKE_EXTENSION_MOTOR = MotorFactory.createSpark("INTAKE_EXTENSION_MOTOR", INTAKE_EXTENSION_MOTOR_PORT, FLEX);

    static final double MINIMUM_INTAKE_SPEED_TANGENTIAL_MPS = 3;
    static final double INTAKE_WHEEL_DIAMETER_METERS = 0.04; //TODO TUNE

    static final double INTAKE_RETRACTED_POSITION = 0;
    static final double INTAKE_DEPLOYED_POSITION = 0.5;

    static {
        configureIntakeRollerMotor();
        configureIntakeExtensionMotor();
    }

    private static void configureIntakeExtensionMotor() {
        final MotorConfiguration config = new MotorConfiguration();

        config.idleMode = MotorProperties.IdleMode.BRAKE;
        config.gearRatio = 1; //todo tu ne

        config.slot = new MotorProperties.Slot(10, 0, 0, 0, 0, 0);//todo: TUNE position control, sysid

        config.forwardSoftLimit = INTAKE_DEPLOYED_POSITION;
        config.reverseSoftLimit = INTAKE_RETRACTED_POSITION;

        config.closedLoopTolerance = 0.02; //todo tune lmao;
        config.profileMaxVelocity = 100;
        config.profileMaxAcceleration = 100; //todo tune too. we want a trap profile.

        config.simulationSlot = new MotorProperties.Slot(1, 0, 0, 0, 0, 0);
        config.simulationProperties = new SimulationProperties.Slot(
                SimulationProperties.SimulationType.SIMPLE_MOTOR,
                DCMotor.getNeoVortex(1),
                1,
                0.2);

        INTAKE_EXTENSION_MOTOR.configure(config);

        INTAKE_EXTENSION_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);
        INTAKE_EXTENSION_MOTOR.setupSignalUpdates(MotorSignal.CURRENT);
        INTAKE_EXTENSION_MOTOR.setupSignalUpdates(MotorSignal.POSITION);
        INTAKE_EXTENSION_MOTOR.setupSignalUpdates(MotorSignal.VELOCITY);
        INTAKE_EXTENSION_MOTOR.setupSignalUpdates(MotorSignal.CLOSED_LOOP_TARGET);
    }

    private static void configureIntakeRollerMotor() {
        final MotorConfiguration config = new MotorConfiguration();

        config.idleMode = MotorProperties.IdleMode.COAST;
        config.inverted = true;

        config.slot = new MotorProperties.Slot(10, 0, 0, 0, 0, 0);
        //todo: TUNE velocity controller, sysid

        config.simulationSlot = new MotorProperties.Slot(1, 0, 0, 0, 0, 0);
        config.simulationProperties = new SimulationProperties.Slot(
                SimulationProperties.SimulationType.SIMPLE_MOTOR,
                DCMotor.getNeoVortex(1),
                1,
                0.2);

        INTAKE_ROLLER_MOTOR.configure(config);

        INTAKE_ROLLER_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);
        INTAKE_ROLLER_MOTOR.setupSignalUpdates(MotorSignal.VELOCITY);
        INTAKE_ROLLER_MOTOR.setupSignalUpdates(MotorSignal.CLOSED_LOOP_TARGET);
    }
}