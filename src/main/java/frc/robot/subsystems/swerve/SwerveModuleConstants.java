package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.generic.hardware.encoder.*;
import frc.lib.generic.hardware.motor.Motor;
import frc.lib.generic.hardware.motor.MotorConfiguration;
import frc.lib.generic.hardware.motor.MotorFactory;
import frc.lib.generic.hardware.motor.MotorProperties;
import frc.lib.generic.simulation.SimProperties;

import static edu.wpi.first.math.system.plant.DCMotor.getKrakenX60Foc;
import static frc.lib.generic.hardware.motor.MotorSignal.*;
import static frc.lib.generic.simulation.SimProperties.SimulationType.SIMPLE_MOTOR;
import static frc.robot.subsystems.swerve.SwerveConstants.DRIVE_GEAR_RATIO;
import static frc.robot.subsystems.swerve.SwerveConstants.STEER_GEAR_RATIO;
import static frc.robot.utilities.PortsConstants.SwervePorts.*;

public class SwerveModuleConstants {
    public static final double
            ROBOT_MODULE_LENGTH_X = 0.55816521,
            ROBOT_MODULE_LENGTH_Y = 0.54546555;

    static final MotorConfiguration steerMotorConfig = new MotorConfiguration();
    static final MotorConfiguration driveMotorConfig = new MotorConfiguration();

    static final MotorProperties.IdleMode ANGLE_NEUTRAL_MODE = MotorProperties.IdleMode.BRAKE;
    static final MotorProperties.IdleMode DRIVE_NEUTRAL_MODE = MotorProperties.IdleMode.BRAKE;

    static final double OPEN_LOOP_RAMP = 0.1;
    static final double CLOSED_LOOP_RAMP = 0.1;

    static final boolean CAN_CODER_INVERT = false;
    static final boolean ANGLE_MOTOR_INVERT = true;
    static final boolean DRIVE_MOTOR_INVERT = false;

    static final int ANGLE_CURRENT_LIMIT = 30;
    public static final int DRIVE_STATOR_CURRENT_LIMIT = 60;

    static final MotorProperties.Slot DRIVE_SLOT = new MotorProperties.Slot(
            0, 0.0, 0.0, //TODO: tune FF values for drive motor.
            0.82849,
            0.08223,
            0.056002);

    protected static final Motor
            FL_STEER_MOTOR = MotorFactory.createTalonFX("FL_STEER_MOTOR", FL_STEER_MOTOR_PORT),
            FR_STEER_MOTOR = MotorFactory.createTalonFX("FR_STEER_MOTOR", FR_STEER_MOTOR_PORT),
            RL_STEER_MOTOR = MotorFactory.createTalonFX("RL_STEER_MOTOR", RL_STEER_MOTOR_PORT),
            RR_STEER_MOTOR = MotorFactory.createTalonFX("RR_STEER_MOTOR", RR_STEER_MOTOR_PORT);

    protected static final Motor
            FL_DRIVE_MOTOR = MotorFactory.createTalonFX("FL_DRIVE_MOTOR", FL_DRIVE_MOTOR_PORT),
            FR_DRIVE_MOTOR = MotorFactory.createTalonFX("FR_DRIVE_MOTOR", FR_DRIVE_MOTOR_PORT),
            RL_DRIVE_MOTOR = MotorFactory.createTalonFX("RL_DRIVE_MOTOR", RL_DRIVE_MOTOR_PORT),
            RR_DRIVE_MOTOR = MotorFactory.createTalonFX("RR_DRIVE_MOTOR", RR_DRIVE_MOTOR_PORT);

    protected static final Encoder
            FL_STEER_ENCODER = EncoderFactory.createCanCoder("FL_STEER_ENCODER", FL_STEER_ENCODER_PORT),
            FR_STEER_ENCODER = EncoderFactory.createCanCoder("FR_STEER_ENCODER", FR_STEER_ENCODER_PORT),
            RL_STEER_ENCODER = EncoderFactory.createCanCoder("RL_STEER_ENCODER", RL_STEER_ENCODER_PORT),
            RR_STEER_ENCODER = EncoderFactory.createCanCoder("RR_STEER_ENCODER", RR_STEER_ENCODER_PORT);

    //fl fr rl rr
    static final double[] STEER_ENCODER_OFFSET = {
            0.542236,0.430664,-0.216309+0.5 ,0.740723 //todo: not completely straight, retune && fix foc error at beginning
    };

    static final Encoder[] STEER_ENCODERS = {FL_STEER_ENCODER, FR_STEER_ENCODER, RL_STEER_ENCODER, RR_STEER_ENCODER};
    static final Motor[] STEER_MOTORS = {FL_STEER_MOTOR, FR_STEER_MOTOR, RL_STEER_MOTOR, RR_STEER_MOTOR};
    static final Motor[] DRIVE_MOTORS = {FL_DRIVE_MOTOR, FR_DRIVE_MOTOR, RL_DRIVE_MOTOR, RR_DRIVE_MOTOR};

    static {
        configureSteerConfiguration();
        configureDriveConfiguration();

        for (int i = 0; i < 4; i++) {
            configureDriveMotor(DRIVE_MOTORS[i]);

            configureSteerEncoder(STEER_ENCODERS[i], Rotation2d.fromRotations(STEER_ENCODER_OFFSET[i]));
            configureSteerMotor(STEER_MOTORS[i], STEER_ENCODERS[i]);
            setSimulatedEncoderSources(STEER_ENCODERS[i], STEER_MOTORS[i]);
        }
    }

    protected static final SwerveModule[] MODULES = new SwerveModule[]{
            new SwerveModule(FL_DRIVE_MOTOR, FL_STEER_MOTOR),
            new SwerveModule(FR_DRIVE_MOTOR, FR_STEER_MOTOR),
            new SwerveModule(RL_DRIVE_MOTOR, RL_STEER_MOTOR),
            new SwerveModule(RR_DRIVE_MOTOR, RR_STEER_MOTOR)
    };

    /**
     * To calculate the angle offset, place all code offsets as 0, zero the modules, and write down the raw position encoder values.
     * The value of the encoders are your offsets
     */
    private static void configureSteerEncoder(Encoder steerEncoder, Rotation2d angleOffset) {
        final EncoderConfiguration encoderConfiguration = new EncoderConfiguration();

        encoderConfiguration.invert = CAN_CODER_INVERT;
        encoderConfiguration.sensorRange = EncoderProperties.SensorRange.NEGATIVE_HALF_TO_HALF;
        encoderConfiguration.offsetRotations = angleOffset.getRotations();

        steerEncoder.configure(encoderConfiguration);

        steerEncoder.setupSignalUpdates(EncoderSignal.POSITION_AND_VELOCITY);
    }


    private static void setSimulatedEncoderSources(Encoder steerEncoder, Motor simulationSource) {
        steerEncoder.setSimulatedEncoderPositionSource(simulationSource::getSystemPosition);
        steerEncoder.setSimulatedEncoderVelocitySource(simulationSource::getSystemVelocity);
    }

    private static void configureDriveMotor(Motor driveMotor) {
        driveMotor.configure(driveMotorConfig);

        driveMotor.setupSignalUpdates(POSITION_AND_VELOCITY, true);

        driveMotor.setupSignalUpdates(CLOSED_LOOP_TARGET);
        driveMotor.setupSignalUpdates(VOLTAGE);
        driveMotor.setupSignalUpdates(ACCELERATION);
        driveMotor.setupSignalUpdates(CURRENT);
    }

    private static void configureSteerMotor(Motor steerMotor, Encoder encoder) {
        steerMotorConfig.remoteSensorDeviceID = encoder.getDeviceID();

        steerMotor.configure(steerMotorConfig);

        steerMotor.setupSignalUpdates(POSITION_AND_VELOCITY, true);

        steerMotor.setupSignalUpdates(VOLTAGE);
        steerMotor.setupSignalUpdates(CLOSED_LOOP_TARGET);

        steerMotor.setMotorEncoderPosition(encoder.getEncoderPosition());
    }

    private static void configureDriveConfiguration() {
        driveMotorConfig.idleMode = DRIVE_NEUTRAL_MODE;
        driveMotorConfig.inverted = DRIVE_MOTOR_INVERT;

        driveMotorConfig.gearRatio = DRIVE_GEAR_RATIO;

        driveMotorConfig.statorCurrentLimit = DRIVE_STATOR_CURRENT_LIMIT;

        driveMotorConfig.slot = DRIVE_SLOT;

        driveMotorConfig.dutyCycleOpenLoopRampPeriod = OPEN_LOOP_RAMP;
        driveMotorConfig.dutyCycleClosedLoopRampPeriod = CLOSED_LOOP_RAMP;

        driveMotorConfig.simulationProperties = new SimProperties.Slot(SIMPLE_MOTOR, getKrakenX60Foc(1), DRIVE_GEAR_RATIO, 0.003);
        driveMotorConfig.simulationSlot = new MotorProperties.Slot(0, 0, 0, 0.74095, 0.019661, 0.010919);
    }

    private static void configureSteerConfiguration() {
        steerMotorConfig.slot = new MotorProperties.Slot(37, 0, 0.05, 0, 0, 0);

        steerMotorConfig.supplyCurrentLimit = ANGLE_CURRENT_LIMIT;
        steerMotorConfig.inverted = ANGLE_MOTOR_INVERT;
        steerMotorConfig.idleMode = ANGLE_NEUTRAL_MODE;

        steerMotorConfig.rotorToSensorRatio = STEER_GEAR_RATIO;

        steerMotorConfig.closedLoopContinuousWrap = true;

        steerMotorConfig.simulationProperties = new SimProperties.Slot(SIMPLE_MOTOR, getKrakenX60Foc(1), STEER_GEAR_RATIO, 0.003);
        steerMotorConfig.simulationSlot = new MotorProperties.Slot(120, 0, 0, 0, 0, 0);
    }
}
