package frc.robot.subsystems.intake;


import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.hardware.motor.*;
import frc.lib.generic.simulation.SimProperties;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.lib.generic.hardware.motor.MotorProperties.SparkType.FLEX;
import static frc.robot.utilities.PortsConstants.IntakePorts.*;

public class IntakeConstants {
    protected static final SysIdRoutine.Config SYSID_EXTENSION_CONFIG = new SysIdRoutine.Config(
            Volts.per(Second).of(0.75),
            Volts.of(3),
            Second.of(5)
    );

    protected static final Motor INTAKE_ROLLER_MOTOR = MotorFactory.createSpark("INTAKE_ROLLER_MOTOR", INTAKE_ROLLER_MOTOR_PORT, FLEX);
    protected static final Motor INTAKE_EXTENSION_MOTOR = MotorFactory.createSpark("INTAKE_EXTENSION_MOTOR", INTAKE_EXTENSION_MOTOR_PORT, FLEX);

    static final double MINIMUM_INTAKE_SPEED_TANGENTIAL_MPS = 3;
    static final double INTAKE_WHEEL_DIAMETER_METERS = 0.041;

    static final double INTAKE_RETRACTED_POSITION = 0;
    static final double INTAKE_DEPLOYED_POSITION = 2.8;

    static {
        configureIntakeRollerMotor();
        configureIntakeExtensionMotor();
    }

    private static void configureIntakeExtensionMotor() {
        final MotorConfiguration config = new MotorConfiguration();

        config.idleMode = MotorProperties.IdleMode.COAST;
        config.gearRatio = 15;

        config.slot = new MotorProperties.Slot(0, 0, 0, 1.5897, 0, 0.090781);
        config.inverted = true;

        config.forwardSoftLimit = INTAKE_DEPLOYED_POSITION;
        config.reverseSoftLimit = INTAKE_RETRACTED_POSITION;

        config.supplyCurrentLimit = 30;
        config.closedLoopTolerance = 0.02;

        config.profileMaxVelocity = 5;
        config.profileMaxAcceleration = 8;

        config.simulationSlot = new MotorProperties.Slot(1, 0, 0, 0, 0, 0);
        config.simulationProperties = new SimProperties.Slot(
                SimProperties.SimulationType.SIMPLE_MOTOR,
                DCMotor.getNeoVortex(1),
                1,
                0.2);

        INTAKE_EXTENSION_MOTOR.configure(config);

        INTAKE_EXTENSION_MOTOR.setMotorEncoderPosition(0);

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

        config.slot = new MotorProperties.Slot(0, 0, 0, 0.23532, 0, 0); //todo test lul, control might not be needed.
        config.gearRatio = 2;
        config.supplyCurrentLimit = 40;

        config.simulationSlot = new MotorProperties.Slot(1, 0, 0, 0, 0, 0);
        config.simulationProperties = new SimProperties.Slot(
                SimProperties.SimulationType.SIMPLE_MOTOR,
                DCMotor.getNeoVortex(1),
                1,
                0.2);

        INTAKE_ROLLER_MOTOR.configure(config);

        INTAKE_ROLLER_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);
        INTAKE_ROLLER_MOTOR.setupSignalUpdates(MotorSignal.VELOCITY);
        INTAKE_ROLLER_MOTOR.setupSignalUpdates(MotorSignal.CLOSED_LOOP_TARGET);
    }
}