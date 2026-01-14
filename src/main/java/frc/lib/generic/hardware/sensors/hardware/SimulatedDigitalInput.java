package frc.lib.generic.hardware.sensors.hardware;

import frc.lib.generic.hardware.sensors.Sensor;
import frc.lib.generic.hardware.sensors.SensorInputsAutoLogged;

public class SimulatedDigitalInput extends Sensor {
    public SimulatedDigitalInput(String name) {
        super(name);
    }

    @Override
    public void refreshInputs(SensorInputsAutoLogged inputs) {
        inputs.currentValue = 1;

        inputs.threadCurrentValue = new int[]{inputs.currentValue};
    }
}
