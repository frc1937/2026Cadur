package frc.lib.generic.simulation.extensions;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ExtendedElevatorSim extends ElevatorSim {
    public ExtendedElevatorSim(
            DCMotor gearbox,
            double gearing,
            double carriageMassKg,
            double drumRadiusMeters,
            double minHeightMeters,
            double maxHeightMeters,
            boolean simulateGravity,
            double startingHeightMeters) {
        super(gearbox, gearing, carriageMassKg, drumRadiusMeters, minHeightMeters, maxHeightMeters, simulateGravity, startingHeightMeters);
    }

    /**
     * Returns the acceleration of the elevator.
     *
     * @return The acceleration of the elevator in meters per second squared.
     */
    public double getAccelerationMetersPerSecondSquared() {
        // Get the current state vector (position and velocity)
        Matrix<N2, N1> x = m_x;

        // Get the input voltage
        Matrix<N1, N1> u = m_u;

        Matrix<N2, N1> xdot = m_plant.getA().times(x).plus(m_plant.getB().times(u));

        return xdot.get(1, 0);
    }
}