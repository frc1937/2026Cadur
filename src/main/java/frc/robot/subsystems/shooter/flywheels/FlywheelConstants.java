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
            MASTER_FLYWHEEL_MOTOR = MotorFactory.createTalonFX("MASTER_LEFT_FLYWHEEL_MOTOR", LEFT_FLYWHEEL_PORT),
            SLAVE_FLYWHEEL_MOTOR = MotorFactory.createTalonFX("SLAVE_RIGHT_FLYWHEEL_MOTOR", RIGHT_FLYWHEEL_PORT);

    protected static final SpeedMechanism2d FLYWHEEL_MECHANISM = createSpeedMechanism("Flywheel Mechanism");

    protected static final double FLYWHEEL_SHOOTING_SPEED_TOLERANCE_RPS = 5; //TODO 300 RPM is way too big.. tune!

    static {
        configureFlywheelMotors();
    }

    private static void configureFlywheelMotors() {
        final MotorConfiguration configuration = new MotorConfiguration();

        configuration.slot = new MotorProperties.Slot(0, 0, 0, 0.1276, 0, 0); //TODO TUNE

        //max speed: 94 rps
        //max acceleration: 44

        configuration.idleMode = MotorProperties.IdleMode.COAST;
        configuration.statorCurrentLimit = 70;
        configuration.closedLoopTolerance = FLYWHEEL_SHOOTING_SPEED_TOLERANCE_RPS; //ROTATIONS PER SEC TODO TUNE

        configuration.simulationSlot = new MotorProperties.Slot(0, 0, 0, 0.1132075472, 0, 0);
        configuration.simulationProperties = new SimProperties.Slot(SIMPLE_MOTOR, getFalcon500(2), 1, 0.002);

        MASTER_FLYWHEEL_MOTOR.configure(configuration);
        SLAVE_FLYWHEEL_MOTOR.configure(configuration);

        MASTER_FLYWHEEL_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);
        MASTER_FLYWHEEL_MOTOR.setupSignalUpdates(MotorSignal.VELOCITY);
        MASTER_FLYWHEEL_MOTOR.setupSignalUpdates(MotorSignal.CLOSED_LOOP_TARGET);
        MASTER_FLYWHEEL_MOTOR.setupSignalUpdates(MotorSignal.ACCELERATION);

        SLAVE_FLYWHEEL_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);//TODO: Check if needed
        SLAVE_FLYWHEEL_MOTOR.setupSignalUpdates(MotorSignal.VELOCITY);//TODO: Check if needed
        SLAVE_FLYWHEEL_MOTOR.setupSignalUpdates(MotorSignal.CLOSED_LOOP_TARGET); //TODO: Check if needed

        SLAVE_FLYWHEEL_MOTOR.setFollowerOf(MASTER_FLYWHEEL_MOTOR, true);
    }
}