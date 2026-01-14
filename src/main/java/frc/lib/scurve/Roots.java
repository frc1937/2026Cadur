package frc.lib.scurve;

import java.util.TreeSet;

import static edu.wpi.first.math.MathUtil.clamp;

public class Roots {
    // Use TreeSet instead of custom Set class
    public static class PositiveSet extends TreeSet<Double> {
        @Override
        public boolean add(Double value) {
            if (value >= 0) {
                return super.add(value);
            }
            return false;
        }
    }

    // Solve resolvent equation of corresponding Quartic equation
    public static double[] solveResolvent(double[] x, double a, double b, double c) {
        final double cos120 = -0.50;
        final double sin120 = 0.866025403784438646764;

        a /= 3;
        double a2 = a * a;
        double q = a2 - b / 3;
        double r = (a * (2 * a2 - b) + c) / 2;
        double r2 = r * r;
        double q3 = q * q * q;

        if (r2 < q3) {
            double qsqrt = Math.sqrt(q);
            double t = clamp(r / (q * qsqrt), -1.0, 1.0);
             q = -2 * qsqrt;

            double theta = Math.acos(t) / 3;
            double ux = Math.cos(theta) * q;
            double uyi = Math.sin(theta) * q;
            x[0] = ux - a;
            x[1] = ux * cos120 - uyi * sin120 - a;
            x[2] = ux * cos120 + uyi * sin120 - a;
            return new double[]{3, x[0], x[1], x[2]};
        }

        double A = -Math.cbrt(Math.abs(r) + Math.sqrt(r2 - q3));
        if (r < 0.0) {
            A = -A;
        }
        double B = (0.0 == A ? 0.0 : q / A);

        x[0] = (A + B) - a;
        x[1] = -(A + B) / 2 - a;
        x[2] = Math.sqrt(3) * (A - B) / 2;
        if (Math.abs(x[2]) < Double.MIN_VALUE) {
            x[2] = x[1];
            return new double[]{2, x[0], x[1], x[2]};
        }

        return new double[]{1, x[0], x[1], x[2]};
    }

    // Calculate all roots of the monic quartic equation: x^4 + a*x^3 + b*x^2 + c*x + d = 0
    public static PositiveSet solveQuartMonic(double a, double b, double c, double d) {
        PositiveSet roots = new PositiveSet();

        if (Math.abs(d) < Double.MIN_VALUE) {
            if (Math.abs(c) < Double.MIN_VALUE) {
                roots.add(0.0);

                double D = a * a - 4 * b;
                if (Math.abs(D) < Double.MIN_VALUE) {
                    roots.add(-a / 2);
                } else if (D > 0.0) {
                    double sqrtD = Math.sqrt(D);
                    roots.add((-a - sqrtD) / 2);
                    roots.add((-a + sqrtD) / 2);
                }
                return roots;
            }

            if (Math.abs(a) < Double.MIN_VALUE && Math.abs(b) < Double.MIN_VALUE) {
                roots.add(0.0);
                roots.add(-Math.cbrt(c));
                return roots;
            }
        }

        double a3 = -b;
        double b3 = a * c - 4 * d;
        double c3 = -a * a * d - c * c + 4 * b * d;

        double[] x3 = new double[3];

        double[] results = solveResolvent(x3, a3, b3, c3);

        int numberZeroes = (int) results[0];

        x3[0] = results[1];
        x3[1] = results[2];
        x3[2] = results[3];

        double y = x3[0];
        // Choosing Y with maximal absolute value.
        if (numberZeroes != 1) {
            if (Math.abs(x3[1]) > Math.abs(y)) {
                y = x3[1];
            }
            if (Math.abs(x3[2]) > Math.abs(y)) {
                y = x3[2];
            }
        }

        double q1, q2, p1, p2;

        double D = y * y - 4 * d;
        if (Math.abs(D) < Double.MIN_VALUE) {
            q1 = q2 = y / 2;
            D = a * a - 4 * (b - y);
            if (Math.abs(D) < Double.MIN_VALUE) {
                p1 = p2 = a / 2;
            } else {
                double sqrtD = Math.sqrt(D);
                p1 = (a + sqrtD) / 2;
                p2 = (a - sqrtD) / 2;
            }
        } else {
            double sqrtD = Math.sqrt(D);
            q1 = (y + sqrtD) / 2;
            q2 = (y - sqrtD) / 2;
            p1 = (a * q1 - c) / (q1 - q2);
            p2 = (c - a * q2) / (q1 - q2);
        }

        final double eps = 16 * Double.MIN_VALUE;

        D = p1 * p1 - 4 * q1;
        if (Math.abs(D) < eps) {
            roots.add(-p1 / 2);
        } else if (D > 0.0) {
            double sqrtD = Math.sqrt(D);
            roots.add((-p1 - sqrtD) / 2);
            roots.add((-p1 + sqrtD) / 2);
        }

        D = p2 * p2 - 4 * q2;
        if (Math.abs(D) < eps) {
            roots.add(-p2 / 2);
        } else if (D > 0.0) {
            double sqrtD = Math.sqrt(D);
            roots.add((-p2 - sqrtD) / 2);
            roots.add((-p2 + sqrtD) / 2);
        }

        return roots;
    }

    // Calculate the quartic equation: x^4 + b*x^3 + c*x^2 + d*x + e = 0
    public static PositiveSet solveQuartMonic(double[] polynom) {
        return solveQuartMonic(polynom[0], polynom[1], polynom[2], polynom[3]);
    }
}
