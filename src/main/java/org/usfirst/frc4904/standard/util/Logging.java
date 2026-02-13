package org.usfirst.frc4904.standard.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

public final class Logging {

    private Logging() {
        throw new UnsupportedOperationException("Cannot instantiate utility class.");
    }

    private static final Map<String, Double> lastLogTimes = new HashMap<>();

    /**
     * @return True if at least {@code delaySeconds} seconds have passed since this function
     *         was last called with the same {@code key}.
     */
    public static boolean cooldown(String key, double delaySeconds) {
        double now = Timer.getFPGATimestamp();
        double prev = lastLogTimes.getOrDefault(key, Double.NEGATIVE_INFINITY);

        if (now - prev >= delaySeconds) {
            lastLogTimes.put(key, now);
            return true;
        } else return false;
    }

    /**
     * Log values with a cooldown to not flood the RioLog™. Formatted as 'key: value' or 'key: [val1, val2, ...]'
     *
     * @param key    Key that is used to determine the cooldown since the last message with the same key was sent
     * @param values Value(s) to log
     * @return Whether the value was logged or skipped on cooldown
     */
    public static boolean log(String key, Object... values) {
        return logWithDelay(key, values.length == 1 ? values[0] : values, 0.5);
    }

    /**
     * Log values with a cooldown to not flood the RioLog™. Formatted as 'key: value'
     *
     * @param key          Key that is used to determine the cooldown since the last message with the same key was sent
     * @param value        Value to log
     * @param delaySeconds Minimum cooldown between logs
     * @return Whether the value was logged or skipped on cooldown
     */
    public static boolean logWithDelay(String key, Object value, double delaySeconds) {
        if (!cooldown(key, delaySeconds)) return false;

        String str;
        if (value == null) {
            str = "null";
        } else if (value.getClass().isArray()) {
            // outermost array passed to deepToString can't be primitive, but nested ones can
            String arrayStr = Arrays.deepToString(new Object[] { value });
            str = arrayStr.substring(1, arrayStr.length() - 1);
        } else if (value instanceof Pose2d pose) {
            // easier to read - default toString does Pose2d(Translation2d(...), Rotation2d(...))
            str = String.format("Pose2d(X: %.2f, Y: %.2f, Rot: %.1fdeg)", pose.getX(), pose.getY(), pose.getRotation().getDegrees());
        } else {
            str = value.toString();
        }

        System.out.println(key + ": " + str);
        return true;
    }
}
