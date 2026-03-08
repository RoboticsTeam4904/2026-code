package org.usfirst.frc4904.standard.silly;
import org.usfirst.frc4904.standard.util.Logging;

public final class Console {
    public static void log(String key, Object... values) {
        Logging.log(key, values);
    }
    public static void logWithDelay(String key, double delaySeconds, Object... values) {
        Logging.log(key, delaySeconds, values);
    }
}

