package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.Feedforward;
import frc.lib.generic.hardware.motor.*;
import frc.lib.generic.simulation.SimulationProperties;
import frc.lib.generic.visualization.mechanisms.MechanismFactory;
import frc.lib.generic.visualization.mechanisms.SingleJointedArmMechanism2d;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.utilities.PortsConstants.HoodPorts.HOOD_MOTOR_PORT;

public class HoodConstants {
    protected static final SysIdRoutine.Config SYSID_HOOD_CONFIG = new SysIdRoutine.Config(
            Volts.per(Second).of(1),
            Volts.of(2),
            Second.of(5)
    );

    protected static final Motor HOOD_MOTOR = MotorFactory.createTalonFX("Hood Motor", HOOD_MOTOR_PORT);
    protected static final SingleJointedArmMechanism2d HOOD_MECHANISM = MechanismFactory.createSingleJointedArmMechanism("Hood Mechanism", 0.5);

    private static final Rotation2d
            HOOD_MINIMUM_ROTATION = Rotation2d.fromDegrees(30),
            HOOD_MAXIMUM_ROTATION = Rotation2d.fromDegrees(85);

    static {
        configureHoodMotorConfiguration();
    }

    private static void configureHoodMotorConfiguration() {
        final MotorConfiguration hoodMotorConfiguration = new MotorConfiguration();

        hoodMotorConfiguration.idleMode = MotorProperties.IdleMode.BRAKE;

        hoodMotorConfiguration.slot = new MotorProperties.Slot(0.145, 0, 0, 0.084973, 0, 0.13081, 0, Feedforward.Type.ARM);

        hoodMotorConfiguration.simulationSlot = new MotorProperties.Slot(40, 0, 1, 0, 0, 0);
        hoodMotorConfiguration.simulationProperties = new SimulationProperties.Slot(
                SimulationProperties.SimulationType.ARM,
                DCMotor.getFalcon500(1),
                1,
                0.5,
                0.01,
                HOOD_MINIMUM_ROTATION,
                HOOD_MAXIMUM_ROTATION,
                true);

        HOOD_MOTOR.configure(hoodMotorConfiguration);

        HOOD_MOTOR.setMotorEncoderPosition(0);

        HOOD_MOTOR.setupSignalUpdates(MotorSignal.POSITION);
        HOOD_MOTOR.setupSignalUpdates(MotorSignal.VELOCITY);
        HOOD_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);
        HOOD_MOTOR.setupSignalUpdates(MotorSignal.CURRENT);
        HOOD_MOTOR.setupSignalUpdates(MotorSignal.CLOSED_LOOP_TARGET);
    }
}