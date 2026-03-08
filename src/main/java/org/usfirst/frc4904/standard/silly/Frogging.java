package org.usfirst.frc4904.standard.silly;
import org.usfirst.frc4904.standard.util.Logging;

public final class Frogging {
    public static <T> T frog(String key, T value, Object... others) {
        return Logging.log(key, value, others);
    }
    public static <T> T frogWithDelay(String key, double delaySeconds, T value, Object... others) {
        return Logging.logWithDelay(key, delaySeconds, value, others);
    }
}
