package frc.robot.subsystems.turret.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.encoder.Encoder;
import frc.lib.generic.hardware.encoder.EncoderFactory;
import frc.lib.generic.hardware.motor.*;
import frc.lib.generic.simulation.SimulationProperties;
import frc.lib.generic.visualization.mechanisms.MechanismFactory;
import frc.lib.generic.visualization.mechanisms.SingleJointedArmMechanism2d;

import static frc.robot.utilities.PortsConstants.TurretPorts.*;

public class TurretConstants extends GenericSubsystem {
    protected static final Motor TURRET_MOTOR = MotorFactory.createTalonFX("Turret Motor", TURRET_MOTOR_PORT);
    protected static final Motor SECOND_TURRET_MOTOR = MotorFactory.createTalonFX("Second Turret Motor", SECOND_TURRET_PORT);
    protected static final Encoder TURRET_ENCODER = EncoderFactory.createCanCoder("Turret Encoder", TURRET_ENCODER_PORT);

    protected static final SingleJointedArmMechanism2d TURRET_MECHANISM = MechanismFactory.createSingleJointedArmMechanism("Turret Mechanism", 5);

    protected static final Rotation2d
            MAX_ANGLE = Rotation2d.fromDegrees(95),
            MIN_ANGLE = Rotation2d.fromDegrees(-95);

    protected static final double
            K_P = 1,
            WHEEL_DIAMETER = 2;

    static {
        configureTurretMotor();
    }

    private static void configureTurretMotor() {
        final MotorConfiguration turretMotorConfiguration = new MotorConfiguration();

        SECOND_TURRET_MOTOR.setFollower(TURRET_MOTOR, true);

        TURRET_MOTOR.setupSignalUpdates(MotorSignal.POSITION);
        TURRET_MOTOR.setupSignalUpdates(MotorSignal.VELOCITY);
        TURRET_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);
        TURRET_MOTOR.setupSignalUpdates(MotorSignal.CLOSED_LOOP_TARGET);

        turretMotorConfiguration.simulationSlot = new MotorProperties.Slot(K_P, 0, 0, 0, 0, 0);
        turretMotorConfiguration.idleMode = MotorProperties.IdleMode.BRAKE;
        turretMotorConfiguration.simulationProperties = new SimulationProperties.Slot(
                SimulationProperties.SimulationType.ARM,
                DCMotor.getFalcon500(1),
                1,
                0.5,
                0.01,
                MIN_ANGLE,
                MAX_ANGLE,
                true);

        TURRET_MOTOR.configure(turretMotorConfiguration);
    }
}