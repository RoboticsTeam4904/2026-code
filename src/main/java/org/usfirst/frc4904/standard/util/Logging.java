package org.usfirst.frc4904.standard.util;

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
        return log(key, values.length == 1 ? values[0] : values, 0.5);
    }

    /**
     * Log values with a cooldown to not flood the RioLog™. Formatted as 'key: value'
     *
     * @param key          Key that is used to determine the cooldown since the last message with the same key was sent
     * @param value        Value to log
     * @param delaySeconds Minimum cooldown between logs
     * @return Whether the value was logged or skipped on cooldown
     */
    public static boolean log(String key, Object value, double delaySeconds) {
        if (!cooldown(key, delaySeconds)) return false;

        if (value != null && value.getClass().isArray()) {
            // outermost array passed to deepToString can't be primitive, but nested ones can
            String str = Arrays.deepToString(new Object[] { value });
            System.out.println(key + ": " + str.substring(1, str.length() - 1));
        } else {
            System.out.println(key + ": " + value);
        }

        return true;
    }
}
