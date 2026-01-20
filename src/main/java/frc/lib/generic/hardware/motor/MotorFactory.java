package frc.lib.generic.hardware.motor;

import frc.lib.generic.hardware.motor.hardware.ctre.GenericTalonFX;
import frc.lib.generic.hardware.motor.hardware.ctre.GenericTalonSRX;
import frc.lib.generic.hardware.motor.hardware.simulated.SimulatedTalonMotor;
import frc.robot.GlobalConstants;

import java.util.ArrayList;
import java.util.List;

import static frc.robot.GlobalConstants.CURRENT_MODE;

public class MotorFactory {
    private static final List<SimulatedTalonMotor> REGISTERED_SIMULATIONS = new ArrayList<>();
    private static final List<Integer> USED_PORTS = new ArrayList<>();

    public static Motor createSpark(String name, int port, MotorProperties.SparkType type) {
        final Motor motor = createSimOrReplayMotor(name, port);

        if (motor != null) return motor;

        return type.getSpark(name, port);
    }

    public static Motor createTalonFX(String name, int port) {
        final Motor motor = createSimOrReplayMotor(name, port);
        if (motor != null) return motor;
        return new GenericTalonFX(name, port);
    }

    public static Motor createTalonSRX(String name, int port) {
        final Motor motor = createSimOrReplayMotor(name, port);

        if (motor != null)
            return motor;

        return new GenericTalonSRX(name, port);
    }

    private static Motor createSimOrReplayMotor(String name, int port) {
        if (CURRENT_MODE == GlobalConstants.Mode.REPLAY)
            return new Motor(name);

        if (CURRENT_MODE == GlobalConstants.Mode.SIMULATION) {
            int adjuster = 0;
            if (USED_PORTS.contains(port)) {
                while (USED_PORTS.contains(port+adjuster)) {
                    adjuster++;
                }
            }

            final SimulatedTalonMotor simulation = new SimulatedTalonMotor(name, port+adjuster);

            USED_PORTS.add(port);
            REGISTERED_SIMULATIONS.add(simulation);
            return simulation;
        }

        return null;
    }

    public static void updateAllSimulations() {
        for (SimulatedTalonMotor simulation : REGISTERED_SIMULATIONS) {
            simulation.updateSimulation();
        }
    }
}
