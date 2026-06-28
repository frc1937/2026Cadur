package frc.robot.subsystems.shooter.kicker;


import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.generic.hardware.motor.*;
import frc.lib.generic.simulation.SimProperties;

import static frc.lib.generic.hardware.motor.MotorProperties.SparkType.MAX;
import static frc.robot.utilities.PortsConstants.KickerPorts.KICKER_MOTOR_PORT;

public class KickerConstants {
    protected static final Motor KICKER_MOTOR = MotorFactory.createSpark("Kicker Motor", KICKER_MOTOR_PORT, MAX);

    static {
        configureKickerMotor();
    }

    private static void configureKickerMotor() {
        final MotorConfiguration config = new MotorConfiguration();

        config.idleMode = MotorProperties.IdleMode.COAST;
        config.supplyCurrentLimit = 40;
        config.closedLoopTolerance = 5;

        config.slot = new MotorProperties.Slot(0.07, 0, 0, 0.132, 0, 0);

        config.simulationSlot = new MotorProperties.Slot(1, 0, 0, 0, 0, 0);
        config.simulationProperties = new SimProperties.Slot(
                SimProperties.SimulationType.SIMPLE_MOTOR,
                DCMotor.getNeoVortex(1),
                1,
                0.001);

        KICKER_MOTOR.configure(config);

        KICKER_MOTOR.setupSignalUpdates(MotorSignal.POSITION);
        KICKER_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);
        KICKER_MOTOR.setupSignalUpdates(MotorSignal.VELOCITY);
        KICKER_MOTOR.setupSignalUpdates(MotorSignal.CURRENT);
        KICKER_MOTOR.setupSignalUpdates(MotorSignal.CLOSED_LOOP_TARGET);
    }
}