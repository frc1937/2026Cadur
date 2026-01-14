package frc.lib.generic.hardware.motor;

import frc.lib.generic.advantagekit.ChoosableLoggedInputs;
import org.littletonrobotics.junction.LogTable;

public class MotorInputs implements ChoosableLoggedInputs {
    public static final int MOTOR_INPUTS_LENGTH = 14;

    public double voltage = 0;
    public double current = 0;
    public double temperature = 0;
    public double target = 0;
    public double systemPosition = 0;
    public double systemVelocity = 0;
    public double systemAcceleration = 0;

    public double[] threadVoltage = new double[0];
    public double[] threadCurrent = new double[0];
    public double[] threadTemperature = new double[0];
    public double[] threadTarget = new double[0];
    public double[] threadSystemPosition = new double[0];
    public double[] threadSystemVelocity = new double[0];
    public double[] threadSystemAcceleration = new double[0];

    private boolean[] signalsToLog;

    @Override
    public void setSignalsToLog(boolean[] signalsToLog) {
        this.signalsToLog = signalsToLog;
    }

    @Override
    public void toLog(LogTable table) {
        if (signalsToLog == null) return;

        if (signalsToLog[0]) table.put("Voltage", voltage);
        if (signalsToLog[1]) table.put("Current", current);
        if (signalsToLog[2]) table.put("Temperature", temperature);
        if (signalsToLog[3]) table.put("Target", target);
        if (signalsToLog[4]) table.put("SystemPosition", systemPosition);
        if (signalsToLog[5]) table.put("SystemVelocity", systemVelocity);
        if (signalsToLog[6]) table.put("SystemAcceleration", systemAcceleration);

        if (signalsToLog[7]) table.put("ThreadVoltage", threadVoltage);
        if (signalsToLog[8]) table.put("ThreadCurrent", threadCurrent);
        if (signalsToLog[9]) table.put("ThreadTemperature", threadTemperature);
        if (signalsToLog[10]) table.put("ThreadTarget", threadTarget);
        if (signalsToLog[11]) table.put("ThreadSystemPosition", threadSystemPosition);
        if (signalsToLog[12]) table.put("ThreadSystemVelocity", threadSystemVelocity);
        if (signalsToLog[13]) table.put("ThreadSystemAcceleration", threadSystemAcceleration);
    }

    @Override
    public void fromLog(LogTable table) {
        voltage = table.get("Voltage", voltage);
        current = table.get("Current", current);
        temperature = table.get("Temperature", temperature);
        target = table.get("Target", target);
        systemPosition = table.get("SystemPosition", systemPosition);
        systemVelocity = table.get("SystemVelocity", systemVelocity);
        systemAcceleration = table.get("SystemAcceleration", systemAcceleration);

        threadVoltage = table.get("ThreadVoltage", threadVoltage);
        threadCurrent = table.get("ThreadCurrent", threadCurrent);
        threadTemperature = table.get("ThreadTemperature", threadTemperature);
        threadTarget = table.get("ThreadTarget", threadTarget);
        threadSystemPosition = table.get("ThreadSystemPosition", threadSystemPosition);
        threadSystemVelocity = table.get("ThreadSystemVelocity", threadSystemVelocity);
        threadSystemAcceleration = table.get("ThreadSystemAcceleration", threadSystemAcceleration);
    }
}
