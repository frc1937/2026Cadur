package frc.lib.generic.advantagekit;

import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface ChoosableLoggedInputs extends LoggableInputs {
    void setSignalsToLog(boolean[] signalsToLog);
}
