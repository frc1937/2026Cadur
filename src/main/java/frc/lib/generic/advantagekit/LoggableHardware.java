package frc.lib.generic.advantagekit;

import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface LoggableHardware extends AutoCloseable {
    /**
     * Call this method periodically
     */
    void periodic();

    /**
     * Get latest sensor input data
     *
     * @return Latest sensor data
     */
    LoggableInputs getInputs();

    @Override
    default void close() {}
}
