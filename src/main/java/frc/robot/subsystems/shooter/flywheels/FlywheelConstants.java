package frc.robot.subsystems.shooter.flywheels;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.generic.hardware.motor.*;
import frc.lib.generic.simulation.SimulationProperties;
import frc.lib.generic.visualization.mechanisms.SpeedMechanism2d;

import static frc.lib.generic.visualization.mechanisms.MechanismFactory.createSpeedMechanism;
import static frc.robot.utilities.PortsConstants.FlywheelPort.LEFT_FLYWHEEL_PORT;
import static frc.robot.utilities.PortsConstants.FlywheelPort.RIGHT_FLYWHEEL_PORT;

public class FlywheelConstants {
    protected static final Motor
            LEFT_FLYWHEEL_MOTOR = MotorFactory.createTalonFX("Left flywheel Motor", LEFT_FLYWHEEL_PORT),
            RIGHT_FLYWHEEL_MOTOR = MotorFactory.createTalonFX("right flywheel Motor", RIGHT_FLYWHEEL_PORT);

    protected static final SpeedMechanism2d
            LEFT_FLYWHEEL_MECHANISM = createSpeedMechanism("Left flywheel Mechanism"),
            RIGHT_FLYWHEEL_MECHANISM = createSpeedMechanism("Left flywheel Mechanism");

    static {
        configureFlywheelMotors();
    }

    private static void configureFlywheelMotors() {
        final MotorConfiguration flywheelMotorConfiguration = new MotorConfiguration();
        flywheelMotorConfiguration.idleMode = MotorProperties.IdleMode.COAST;

        flywheelMotorConfiguration.slot = new MotorProperties.Slot(10, 0, 0, 0, 0, 0);

        flywheelMotorConfiguration.simulationSlot = new MotorProperties.Slot(10, 0, 0, 0, 0, 0);
        flywheelMotorConfiguration.simulationProperties = new SimulationProperties.Slot(
                SimulationProperties.SimulationType.SIMPLE_MOTOR,
                DCMotor.getFalcon500(1),
                150,
                0.2);

        LEFT_FLYWHEEL_MOTOR.configure(flywheelMotorConfiguration);

        flywheelMotorConfiguration.slot = new MotorProperties.Slot(10, 0, 0, 0, 0, 0);
        flywheelMotorConfiguration.inverted = true;

        RIGHT_FLYWHEEL_MOTOR.configure(flywheelMotorConfiguration);

        LEFT_FLYWHEEL_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);
        LEFT_FLYWHEEL_MOTOR.setupSignalUpdates(MotorSignal.VELOCITY);
        LEFT_FLYWHEEL_MOTOR.setupSignalUpdates(MotorSignal.CLOSED_LOOP_TARGET);

        RIGHT_FLYWHEEL_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);
        RIGHT_FLYWHEEL_MOTOR.setupSignalUpdates(MotorSignal.VELOCITY);
        RIGHT_FLYWHEEL_MOTOR.setupSignalUpdates(MotorSignal.CLOSED_LOOP_TARGET);
    }
}