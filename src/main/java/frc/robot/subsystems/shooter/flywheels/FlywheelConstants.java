package frc.robot.subsystems.shooter.flywheels;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.hardware.motor.*;
import frc.lib.generic.simulation.SimProperties;
import frc.lib.generic.visualization.mechanisms.SpeedMechanism2d;

import static edu.wpi.first.math.system.plant.DCMotor.getFalcon500;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.lib.generic.simulation.SimProperties.SimulationType.SIMPLE_MOTOR;
import static frc.lib.generic.visualization.mechanisms.MechanismFactory.createSpeedMechanism;
import static frc.robot.utilities.PortsConstants.FlywheelPort.LEFT_FLYWHEEL_PORT;
import static frc.robot.utilities.PortsConstants.FlywheelPort.RIGHT_FLYWHEEL_PORT;

public class FlywheelConstants {
    protected static final SysIdRoutine.Config SYSID_FLYWHEEL_CONFIG = new SysIdRoutine.Config(
            Volts.per(Second).of(1),
            Volts.of(2),
            Second.of(5)
    );

    protected static final Motor
            MASTER_LEFT_FLYWHEEL_MOTOR = MotorFactory.createTalonFX("MASTER Left flywheel Motor", LEFT_FLYWHEEL_PORT),
            SLAVE_RIGHT_FLYWHEEL_MOTOR = MotorFactory.createTalonFX("SLAVE Right flywheel Motor", RIGHT_FLYWHEEL_PORT);

    protected static final SpeedMechanism2d FLYWHEEL_MECHANISM = createSpeedMechanism("Flywheel Mechanism");

    static {
        configureFlywheelMotors();
    }

    private static void configureFlywheelMotors() {
        final MotorConfiguration configuration = new MotorConfiguration();

        configuration.slot = new MotorProperties.Slot(10, 0, 0, 0, 0, 0); //TODO TUNE

        configuration.idleMode = MotorProperties.IdleMode.COAST;
        configuration.statorCurrentLimit = 70;
        configuration.closedLoopTolerance = 200/60.0; //ROTATIONS PER SEC TODO TUNE

        configuration.simulationSlot = new MotorProperties.Slot(0, 0, 0, 0.1132075472, 0, 0);
        configuration.simulationProperties = new SimProperties.Slot(SIMPLE_MOTOR, getFalcon500(2), 1, 0.002);

        MASTER_LEFT_FLYWHEEL_MOTOR.configure(configuration);
        SLAVE_RIGHT_FLYWHEEL_MOTOR.configure(configuration);

        MASTER_LEFT_FLYWHEEL_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);
        MASTER_LEFT_FLYWHEEL_MOTOR.setupSignalUpdates(MotorSignal.VELOCITY);
        MASTER_LEFT_FLYWHEEL_MOTOR.setupSignalUpdates(MotorSignal.CLOSED_LOOP_TARGET);
        MASTER_LEFT_FLYWHEEL_MOTOR.setupSignalUpdates(MotorSignal.ACCELERATION);

        SLAVE_RIGHT_FLYWHEEL_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);//TODO: Check if needed
        SLAVE_RIGHT_FLYWHEEL_MOTOR.setupSignalUpdates(MotorSignal.VELOCITY);//TODO: Check if needed
        SLAVE_RIGHT_FLYWHEEL_MOTOR.setupSignalUpdates(MotorSignal.CLOSED_LOOP_TARGET); //TODO: Check if needed

        SLAVE_RIGHT_FLYWHEEL_MOTOR.setFollowerOf(MASTER_LEFT_FLYWHEEL_MOTOR, true);
    }
}