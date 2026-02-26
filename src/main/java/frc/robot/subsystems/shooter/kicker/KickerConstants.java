package frc.robot.subsystems.shooter.kicker;


import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.generic.hardware.motor.*;
import frc.lib.generic.simulation.SimProperties;

import static frc.lib.generic.hardware.motor.MotorProperties.SparkType.FLEX;
import static frc.robot.utilities.PortsConstants.KickerPorts.KICKER_MOTOR_PORT;

public class KickerConstants {
    protected static final Motor KICKER_MOTOR = MotorFactory.createSpark("Kicker Motor", KICKER_MOTOR_PORT, FLEX);

    static {
        configureKickerMotor();
    }

    private static void configureKickerMotor() {
        final MotorConfiguration kickerMotorConfiguration = new MotorConfiguration();

        kickerMotorConfiguration.idleMode = MotorProperties.IdleMode.COAST;

        kickerMotorConfiguration.simulationSlot = new MotorProperties.Slot(1, 0, 0, 0, 0, 0);
        kickerMotorConfiguration.simulationProperties = new SimProperties.Slot(
                SimProperties.SimulationType.SIMPLE_MOTOR,
                DCMotor.getNeoVortex(1),
                1,
                0.001);

        KICKER_MOTOR.configure(kickerMotorConfiguration);

        KICKER_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);
    }
}