package frc.lib.generic.simulation;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.generic.simulation.extensions.ExtendedElevatorSim;
import frc.lib.math.Conversions;

import static frc.robot.GlobalConstants.PERIODIC_TIME_SEC;

public class ElevatorSimulation extends GenericPhysicsSimulation {
    private final ExtendedElevatorSim elevatorSimulation;

    private final double minimumHeightMetres;
    private final double drumDiameterMetres;

    public ElevatorSimulation(DCMotor gearbox, double gearRatio, double carriageMassKilograms, double drumRadiusMeters, double minimumHeightMetres, double maximumHeightMetres, boolean simulateGravity) {
        super(gearRatio);

        elevatorSimulation = new ExtendedElevatorSim(
                gearbox,
                gearRatio,
                carriageMassKilograms,
                drumRadiusMeters,
                minimumHeightMetres,
                maximumHeightMetres,
                simulateGravity,
                minimumHeightMetres
        );

        this.minimumHeightMetres = minimumHeightMetres;
        this.drumDiameterMetres = drumRadiusMeters * 2;
    }

    @Override
    public double getCurrent() {
        return elevatorSimulation.getCurrentDrawAmps();
    }

    @Override
    public double getSystemPositionRotations() {
        return Conversions.metresToRotations(
                elevatorSimulation.getPositionMeters() - minimumHeightMetres, drumDiameterMetres);
    }

    @Override
    public double getSystemVelocityRotationsPerSecond() {
        return Conversions.mpsToRps(elevatorSimulation.getVelocityMetersPerSecond(), drumDiameterMetres);
    }

    @Override
    public double getSystemAccelerationRotationsPerSecondSquared() {
        return Conversions.mpsToRps(elevatorSimulation.getAccelerationMetersPerSecondSquared(), drumDiameterMetres);
    }

    @Override
    public void setVoltage(double voltage) {
        elevatorSimulation.setInputVoltage(voltage);
    }

    @Override
    public void updateMotor() {
        elevatorSimulation.update((PERIODIC_TIME_SEC));
    }
}
