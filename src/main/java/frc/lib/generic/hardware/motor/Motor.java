package frc.lib.generic.hardware.motor;

import frc.lib.generic.advantagekit.LoggableHardware;
import frc.lib.generic.hardware.HardwareManager;
import frc.lib.generic.hardware.encoder.Encoder;
import frc.robot.GlobalConstants;
import org.littletonrobotics.junction.Logger;

import java.util.NoSuchElementException;
import java.util.function.DoubleSupplier;

import static frc.lib.generic.hardware.motor.MotorInputs.MOTOR_INPUTS_LENGTH;
import static frc.robot.GlobalConstants.CURRENT_MODE;

/**
 * Custom Motor class to allow switching and replacing motors quickly,
 * in addition of better uniformity across the code.
 */
public class Motor implements LoggableHardware {
    private final MotorInputs inputs = new MotorInputs();
    private final String name;

    private MotorConfiguration configuration;

    public Motor(String name) {
        this.name = "Motors/" + name;

        periodic();
        HardwareManager.addHardware(this);
    }

    public String getName() {
        return name;
    }

    /**
     * Supplies an external position to the feedforward and PID controllers,
     * allowing more precise control using an external {@link Encoder}.
     *
     * @param positionSupplier provides the external position (rotations)
     */
    public void setExternalPositionSupplier(DoubleSupplier positionSupplier) { }


    /**
     * Supplies an external velocity to the feedforward and PID controllers,
     * allowing more precise control using an external {@link Encoder}.
     *
     * @param velocitySupplier provides the external velocity (rotations per second)
     */
    public void setExternalVelocitySupplier(DoubleSupplier velocitySupplier) { }

    /**
     * Sets the motor output using the built-in feedforward and PID controller.
     *
     * <p>Supported control modes:
     * <ul>
     *   <li>{@link MotorProperties.ControlMode#CURRENT} — target current (A)</li>
     *   <li>{@link MotorProperties.ControlMode#VOLTAGE} — target voltage (V)</li>
     *   <li>{@link MotorProperties.ControlMode#POSITION} — target position (rotations)</li>
     *   <li>{@link MotorProperties.ControlMode#VELOCITY} — target velocity (RPS)</li>
     * </ul>
     *
     * <p>For POSITION and VELOCITY modes, a trapezoidal motion profile is used when
     * both {@link MotorConfiguration#profileMaxVelocity} and
     * {@link MotorConfiguration#profileMaxAcceleration} are configured.
     *
     * @param controlMode how the output value is interpreted
     * @param output      the desired output value
     */
    public void setOutput(MotorProperties.ControlMode controlMode, double output) { }

    /**
     * Sets the motor output with a custom feedforward value.
     *
     * <p>Behaves identically to {@link #setOutput(MotorProperties.ControlMode, double)},
     * but applies the given feedforward in POSITION and VELOCITY control modes.
     *
     * @param controlMode the control mode for the motor
     * @param output      the desired output value
     * @param feedforward the custom feedforward to apply (volts)
     */
    public void setOutput(MotorProperties.ControlMode controlMode, double output, double feedforward) { }

    public void ignoreSoftwareLimits(boolean ignoreLimits) { }

    public void setIdleMode(MotorProperties.IdleMode idleMode) {
        getConfig().idleMode = idleMode;
        configure(getConfig());
    }

    public void stopMotor() { }

    /**
     * Resets the motor encoder to the given position.
     *
     * @param position the desired encoder position (rotations)
     */
    public void setMotorEncoderPosition(double position) { }

    public int getDeviceID() { return -1; }

    /**
     * Returns the motor position before gearing is applied.
     *
     * @return position in rotations
     */
    public double getMotorPosition() { return getSystemPosition() / getConfig().gearRatio; }

    /**
     * Returns the motor velocity before gearing is applied.
     *
     * @return velocity in rotations per second
     */
    public double getMotorVelocity() { return getSystemVelocity() / getConfig().gearRatio; }

    /**
     * Get the voltage running through the motor
     *
     * @Units In volts
     */
    public double getVoltage() {
        if (!getSignalsToLog()[0]) printSignalError("VOLTAGE");
        return inputs.voltage;
    }

    /**
     * Get the current running through the motor (STATOR current)
     *
     * @Units In amps
     */
    public double getCurrent() {
        if (!getSignalsToLog()[1]) printSignalError("CURRENT");
        return inputs.current;
    }

    /**
     * Get the temperature of the motor
     * @Units In celsius
     */
    public double getTemperature() {
        if (!getSignalsToLog()[2]) printSignalError("TEMPERATURE");
        return inputs.temperature;
    }

    /**
     * Get the current target of the closed-loop PID
     */
    public double getClosedLoopTarget() {
        if (!getSignalsToLog()[3]) printSignalError("CLOSED_LOOP_TARGET");
        return inputs.target;
    }

    /**
     * Gearing applied
     *
     * @Units In rotations
     */
    public double getSystemPosition() {
        if (!getSignalsToLog()[4]) printSignalError("POSITION");
        return inputs.systemPosition;
    }

    /**
     * Gearing applied
     *
     * @Units In rotations per second
     */
    public double getSystemVelocity() {
        if (!getSignalsToLog()[5]) printSignalError("VELOCITY");
        return inputs.systemVelocity;
    }

    /**
     * Gearing applied
     *
     * @Units In rotations per second
     */
    public double getSystemAcceleration() {
        if (!getSignalsToLog()[6]) printSignalError("ACCELERATION");
        return inputs.systemAcceleration;
    }

    public void setFollowerOf(Motor motor, boolean invert) { }

    /**
     * Registers a signal for automatic periodic logging.
     *
     * @param signal          the signal to log
     * @param useFasterThread whether to update on the high-frequency odometry thread
     */
    public void setupSignalUpdates(MotorSignal signal, boolean useFasterThread) { }

    /**
     * Equivalent to {@link #setupSignalUpdates(MotorSignal, boolean)} with
     * {@code useFasterThread = false}.
     *
     * @param signal the signal to log
     */
    public void setupSignalUpdates(MotorSignal signal) { setupSignalUpdates(signal, false); }

    public boolean configure(MotorConfiguration configuration) {
        this.configuration = configuration;
        return true;
    }

    /**
     * Gets the currently used configuration used by the motor. If this is not set, it will return null.
     *
     * @return The configuration
     */
    public MotorConfiguration getConfig() { return configuration; }

    public boolean isAtPositionSetpoint() {
        if (getConfig() == null || getConfig().closedLoopTolerance == 0)
            new UnsupportedOperationException("You must set the tolerance before checking if the mechanism is at the setpoint.").printStackTrace();

        return Math.abs(getClosedLoopTarget() - getSystemPosition()) < getConfig().closedLoopTolerance;
    }

    public boolean isAtVelocitySetpoint() {
        if (getConfig() == null || getConfig().closedLoopTolerance == 0)
            new UnsupportedOperationException("You must set the tolerance before checking if the mechanism is at the setpoint.").printStackTrace();

        return Math.abs(getClosedLoopTarget() - getSystemVelocity()) < getConfig().closedLoopTolerance;
    }

    protected void refreshInputs(MotorInputs inputs) { }

    protected boolean[] getSignalsToLog() { return new boolean[MOTOR_INPUTS_LENGTH]; }

    @Override
    public void periodic() {
        refreshInputs(inputs);
        Logger.processInputs(name, inputs);
    }

    @Override
    public MotorInputs getInputs() {
        return inputs;
    }

    private void printSignalError(String signalName) {
        if (CURRENT_MODE == GlobalConstants.Mode.REPLAY) return;

        new NoSuchElementException("--------------------\n" +
                "ERROR: TRYING TO RETRIEVE UNINITIALIZED SIGNAL " + signalName + " AT " + getClass().getName() + name +
                "\n--------------------").printStackTrace();
    }
}
