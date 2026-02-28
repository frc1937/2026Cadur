package frc.lib.generic;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;

import static com.ctre.phoenix6.BaseStatusSignal.getLatencyCompensatedValueAsDouble;
import static frc.lib.util.QueueUtilities.queueToDoubleArray;
import static frc.robot.GlobalConstants.*;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This version is intended for devices like the SparkMax that require polling rather than a
 * blocking thread. A Notifier thread is used to gather samples with consistent timing.
 */
public class OdometryThread {
    public static final double ODOMETRY_FREQUENCY_HERTZ = 200.0;

    private static class SignalPair {
        BaseStatusSignal position;
        BaseStatusSignal velocity = null;

        Queue<Double> positionQueue;
        Queue<Double> velocityQueue = null;

        boolean isYawSignal;
    }

    private final List<SignalPair> signalPairs = new ArrayList<>();
    private final Queue<Double> timestamps = new ArrayBlockingQueue<>(100);

    private volatile BaseStatusSignal[] allSignals = new BaseStatusSignal[0];

    private final ThreadInputsAutoLogged threadInputs = new ThreadInputsAutoLogged();

    private static OdometryThread INSTANCE = null;

    public static OdometryThread getInstance() {
        if (INSTANCE == null)
            INSTANCE = new OdometryThread();

        return INSTANCE;
    }

    private OdometryThread() {
        if (CURRENT_MODE == Mode.REPLAY) return;

        Notifier notifier = new Notifier(this::periodic);
        notifier.setName("OdometryThread");

        Timer.delay(5);

        notifier.startPeriodic(1.0 / ODOMETRY_FREQUENCY_HERTZ);
    }

    public Pair<Queue<Double>, Queue<Double>> registerCTRESignalPair(BaseStatusSignal positionSignal, BaseStatusSignal velocitySignal) {
        final Queue<Double> posQueue = new ArrayBlockingQueue<>(100);
        final Queue<Double> velQueue = new ArrayBlockingQueue<>(100);

        FASTER_THREAD_LOCK.lock();
        try {
            insertCTRESignalToSignalArray(positionSignal);
            insertCTRESignalToSignalArray(velocitySignal);

            signalPairs.add(new SignalPair() {{
                position = positionSignal;
                velocity = velocitySignal;
                positionQueue = posQueue;
                velocityQueue = velQueue;
                isYawSignal = positionSignal.getName().equals("Yaw");
            }});

        } finally {
            FASTER_THREAD_LOCK.unlock();
        }

        return new Pair<>(posQueue, velQueue);
    }

    public Queue<Double> registerCTRESignal(BaseStatusSignal signal) {
        Queue<Double> currentQueue = new ArrayBlockingQueue<>(100);
        FASTER_THREAD_LOCK.lock();

        try {
            insertCTRESignalToSignalArray(signal);

            signalPairs.add(new SignalPair() {{
                position = signal;
                positionQueue = currentQueue;
            }});
        } finally {
            FASTER_THREAD_LOCK.unlock();
        }

        return currentQueue;
    }

    private void periodic() {
        if (BaseStatusSignal.refreshAll(allSignals) != StatusCode.OK)
            return;

        final double currentTimestamp = RobotController.getFPGATime() / 1e6;

        FASTER_THREAD_LOCK.lock();
        try {
            for (SignalPair pair : signalPairs) {
                if (pair.velocity == null) {
                    pair.positionQueue.offer(pair.isYawSignal
                            ? pair.position.getValueAsDouble() / 360.0
                            : pair.position.getValueAsDouble());

                    continue;
                }

                pair.positionQueue.offer(getLatencyCompensatedValueAsDouble(pair.position, pair.velocity));
                pair.velocityQueue.offer(pair.velocity.getValueAsDouble());
            }

            timestamps.offer(currentTimestamp);
        } finally {
            FASTER_THREAD_LOCK.unlock();
        }
    }

    private void insertCTRESignalToSignalArray(BaseStatusSignal statusSignal) {
        final BaseStatusSignal[] newSignals = new BaseStatusSignal[allSignals.length + 1];

        System.arraycopy(allSignals, 0, newSignals, 0, allSignals.length);
        newSignals[allSignals.length] = statusSignal;

        allSignals = newSignals;
    }

    public void updateLatestTimestamps() {
        if (CURRENT_MODE != Mode.REPLAY) {
            threadInputs.timestamps = queueToDoubleArray(timestamps);
        }

        Logger.processInputs("OdometryThread", threadInputs);
    }

    public double[] getLatestTimestamps() {
        return threadInputs.timestamps;
    }

    @AutoLog
    public static class ThreadInputs {
        public double[] timestamps;
    }
}