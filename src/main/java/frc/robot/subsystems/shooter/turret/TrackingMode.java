package frc.robot.subsystems.shooter.turret;

public enum TrackingMode {
    AGGRESSIVE(TrackingMode::pickShortest),
    PASSIVE(TrackingMode::pickClosestToZero);

    private final TargetPicker picker;

    TrackingMode(TargetPicker picker) {
        this.picker = picker;
    }

    public double select(double current, double direct, double up, double down, double min, double max) {
        return picker.choose(current, direct, up, down, min, max);
    }

    private static double pickShortest(double currentPos, double direct, double wrapUp, double wrapDown,
                               double min, double max) {
        double best = direct;
        double shortestDist = inRange(direct, min, max) ? Math.abs(direct - currentPos) : Double.MAX_VALUE;

        if (inRange(wrapUp, min, max)) {
            final double dist = Math.abs(wrapUp - currentPos);

            if (dist < shortestDist) {
                shortestDist = dist;
                best = wrapUp;
            }
        }

        if (inRange(wrapDown, min, max) && Math.abs(wrapDown - currentPos) < shortestDist)
            best = wrapDown;

        return best;
    }

    private static double pickClosestToZero(double current, double direct, double wrapUp, double wrapDown, double min, double max) {
        double best = direct;
        double closestToZero = inRange(direct, min, max) ? Math.abs(direct) : Double.MAX_VALUE;

        if (inRange(wrapUp, min, max) && Math.abs(wrapUp) < closestToZero) {
            closestToZero = Math.abs(wrapUp);
            best = wrapUp;
        }

        if (inRange(wrapDown, min, max) && Math.abs(wrapDown) < closestToZero)
            best = wrapDown;

        return best;
    }

    private static boolean inRange(double value, double min, double max) {
        return value >= min && value <= max;
    }

    @FunctionalInterface
    private interface TargetPicker {
        double choose(double current, double direct, double up, double down, double min, double max);
    }
}
