package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.Feedforward;
import frc.lib.generic.hardware.motor.*;
import frc.lib.generic.simulation.SimProperties;
import frc.lib.generic.visualization.mechanisms.MechanismFactory;
import frc.lib.generic.visualization.mechanisms.SingleJointedArmMechanism2d;

import static edu.wpi.first.math.system.plant.DCMotor.getFalcon500;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.lib.generic.simulation.SimProperties.SimulationType.ARM;
import static frc.robot.utilities.PortsConstants.HoodPorts.HOOD_MOTOR_PORT;

public class HoodConstants {
    protected static final SysIdRoutine.Config SYSID_HOOD_CONFIG = new SysIdRoutine.Config(
            Volts.per(Second).of(1),
            Volts.of(2),
            Second.of(5)
    );

    public static final Motor HOOD_MOTOR = MotorFactory.createTalonFX("HOOD_MOTOR", HOOD_MOTOR_PORT);
    protected static final SingleJointedArmMechanism2d HOOD_MECHANISM = MechanismFactory.createSingleJointedArmMechanism("Hood Mechanism", 0.5);

    protected static final double HOOD_ANGLE_TOLERANCE_ROTATIONS = 0.00138;

    protected static final Rotation2d
            MIN_ANGLE = Rotation2d.fromDegrees(11.6),
            PASSING_ANGLE = Rotation2d.fromDegrees(23.6),
            MAX_ANGLE = Rotation2d.fromDegrees(40.6);

    public static final InterpolatingDoubleTreeMap HOOD_ANGLE_TO_SHOOTER_LENGTH = new InterpolatingDoubleTreeMap();

    static {
        configureHoodMotorConfiguration();

        HOOD_ANGLE_TO_SHOOTER_LENGTH.put(MIN_ANGLE.getRotations(), 0.187);
        HOOD_ANGLE_TO_SHOOTER_LENGTH.put(MAX_ANGLE.getRotations(), 0.254);
    }

    private static void configureHoodMotorConfiguration() {
        final MotorConfiguration configuration = new MotorConfiguration();

        configuration.idleMode = MotorProperties.IdleMode.BRAKE;
        configuration.inverted = true;

        configuration.slot = new MotorProperties.Slot(3, 0, 0.1, 11.3, 0, 0.3950, 0, Feedforward.Type.ARM);
        configuration.profileMaxVelocity = 0.97;
        configuration.profileMaxAcceleration = 3;

        configuration.statorCurrentLimit = 35;
        configuration.gearRatio = 102.77748;
        configuration.closedLoopTolerance = HOOD_ANGLE_TOLERANCE_ROTATIONS;

        configuration.forwardSoftLimit = MAX_ANGLE.getRotations();
        configuration.reverseSoftLimit = MIN_ANGLE.getRotations();

        configuration.simulationSlot = new MotorProperties.Slot(0, 0, 0, 11.2240, 0, 0);
        configuration.simulationProperties = new SimProperties.Slot(
                ARM,
                getFalcon500(1),
                100,
                0.20,
                0.1,
                MAX_ANGLE,
                MIN_ANGLE,
                false);

        HOOD_MOTOR.configure(configuration);

        HOOD_MOTOR.setupSignalUpdates(MotorSignal.POSITION);
        HOOD_MOTOR.setupSignalUpdates(MotorSignal.VELOCITY);
        HOOD_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);
        HOOD_MOTOR.setupSignalUpdates(MotorSignal.CURRENT);
        HOOD_MOTOR.setupSignalUpdates(MotorSignal.CLOSED_LOOP_TARGET);
    }
}