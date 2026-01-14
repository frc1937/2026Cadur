package frc.lib.scurve;

import static frc.lib.scurve.Utilities.back;

class Profile {
    static double v_eps = 1e-12;
    static double a_eps = 1e-12;

    static double p_precision = 1e-8;
    static double v_precision = 1e-8;
    static double a_precision = 1e-10;

    public double[] time = new double[7], totalTime = new double[7], j = new double[7];
    public double[] a = new double[8], v = new double[8], p = new double[8];

    //! Target (final) kinematic state
    public double pf;

    public enum ReachedLimits {ACC0_ACC1_VEL, VEL, ACC0, ACC1, ACC0_ACC1, ACC0_VEL, ACC1_VEL, NONE}

    public enum Direction {UP, DOWN}

    public enum ControlSigns {UDDU, UDUD}

    public ReachedLimits limits;
    public Direction direction;
    public ControlSigns control_signs;

    // For third-order position interface
    public boolean check(ControlSigns control_signs, ReachedLimits limits, double jf, double vMax, double vMin, double aMax, double aMin) {
        return check(control_signs, limits, false, jf, vMax, vMin, aMax, aMin);
    }

    public boolean check(ControlSigns control_signs, ReachedLimits limits, boolean set_limits, double jf, double vMax, double vMin, double aMax, double aMin) {
        if (time[0] < 0) return false;

        totalTime[0] = time[0];
        for (int i = 0; i < 6; ++i) {
            if (time[i + 1] < 0) {
                return false;
            }

            totalTime[i + 1] = totalTime[i] + time[i + 1];
        }

        if (limits == ReachedLimits.ACC0_ACC1_VEL || limits == ReachedLimits.ACC0_VEL
                || limits == ReachedLimits.ACC1_VEL || limits == ReachedLimits.VEL) {
            if (time[3] < Double.MIN_VALUE) {
                return false;
            }
        }

        if ((limits == ReachedLimits.ACC0 || limits == ReachedLimits.ACC0_ACC1) && time[1] < Double.MIN_VALUE) {
            return false;
        }


        if ((limits == ReachedLimits.ACC1 || limits == ReachedLimits.ACC0_ACC1) && time[5] < Double.MIN_VALUE) {
            return false;
        }


        if (control_signs == ControlSigns.UDDU) {
            j = new double[]{(time[0] > 0 ? jf : 0), 0, (time[2] > 0 ? -jf : 0), 0, (time[4] > 0 ? -jf : 0), 0, (time[6] > 0 ? jf : 0)};
        } else {
            j = new double[]{(time[0] > 0 ? jf : 0), 0, (time[2] > 0 ? -jf : 0), 0, (time[4] > 0 ? jf : 0), 0, (time[6] > 0 ? -jf : 0)};
        }

        direction = vMax > 0 ? Direction.UP : Direction.DOWN;
        double vUppLim = (direction == Direction.UP ? vMax : vMin) + v_eps;
        double vLowLim = (direction == Direction.UP ? vMin : vMax) - v_eps;

        for (int i = 0; i < 7; ++i) {
            a[i + 1] = a[i] + time[i] * j[i];
            v[i + 1] = v[i] + time[i] * (a[i] + time[i] * j[i] / 2);
            p[i + 1] = p[i] + time[i] * (v[i] + time[i] * (a[i] / 2 + time[i] * j[i] / 6));

            if ((limits == ReachedLimits.ACC0_ACC1_VEL || limits == ReachedLimits.ACC0_ACC1 || limits == ReachedLimits.ACC0_VEL || limits == ReachedLimits.ACC1_VEL || limits == ReachedLimits.VEL) && i == 2) {
                a[3] = 0.0;
            }


            if (set_limits) {
                if (limits == ReachedLimits.ACC1 && i == 2) {
                    a[3] = aMin;
                }


                if (limits == ReachedLimits.ACC0_ACC1) {
                    if (i == 0) {
                        a[1] = aMax;
                    }

                    if (i == 4) {
                        a[5] = aMin;
                    }
                }
            }

            if (i > 1 && a[i + 1] * a[i] < -Double.MIN_VALUE) {
                double v_a_zero = v[i] - (a[i] * a[i]) / (2 * j[i]);
                if (v_a_zero > vUppLim || v_a_zero < vLowLim) {
                    return false;
                }
            }
        }

        this.control_signs = control_signs;
        this.limits = limits;

        double aUppLim = (direction == Direction.UP ? aMax : aMin) + a_eps;
        double aLowLim = (direction == Direction.UP ? aMin : aMax) - a_eps;

        return Math.abs(back(p) - pf) < p_precision && Math.abs(back(v)) < v_precision && Math.abs(back(a)) < a_precision
                && a[1] >= aLowLim && a[3] >= aLowLim && a[5] >= aLowLim
                && a[1] <= aUppLim && a[3] <= aUppLim && a[5] <= aUppLim
                && v[3] <= vUppLim && v[4] <= vUppLim && v[5] <= vUppLim && v[6] <= vUppLim
                && v[3] >= vLowLim && v[4] >= vLowLim && v[5] >= vLowLim && v[6] >= vLowLim;
    }

    void set_boundary(double p0_new, double v0_new, double a0_new, double pf_new) {
        a[0] = a0_new;
        v[0] = v0_new;
        p[0] = p0_new;
        pf = pf_new;
    }
}
