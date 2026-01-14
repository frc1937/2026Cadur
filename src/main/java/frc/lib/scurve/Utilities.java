package frc.lib.scurve;

public class Utilities {

    //! Integrate with constant jerk for duration t. Returns new position, new velocity, and new acceleration.
    public static double[] integrate(double t, double p0, double v0, double a0, double j) {
        return new double[]{
                p0 + t * (v0 + t * (a0 / 2 + t * j / 6)),
                v0 + t * (a0 + t * j / 2),
                a0 + t * j
        };
    }

    public static double back(double[] arr) {
        return arr[arr.length - 1];
    }
}
