package frc.lib.generic.hardware.sensors;

import frc.lib.generic.hardware.sensors.hardware.DigitalInput;
import frc.lib.generic.hardware.sensors.hardware.SimulatedDigitalInput;
import frc.robot.GlobalConstants;

import static frc.robot.GlobalConstants.CURRENT_MODE;

public class SensorFactory {
    public static Sensor createDigitalInput(String name, int port) {
        if (CURRENT_MODE == GlobalConstants.Mode.REPLAY) {
            return new Sensor(name);
        }

        if (CURRENT_MODE == GlobalConstants.Mode.SIMULATION) {
            return new SimulatedDigitalInput(name);
        }

        return new DigitalInput(name, port);
    }
}
