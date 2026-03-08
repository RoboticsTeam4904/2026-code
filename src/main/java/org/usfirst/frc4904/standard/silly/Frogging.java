package org.usfirst.frc4904.standard.silly;
import org.usfirst.frc4904.standard.util.Logging;

public final class Frogging {
    public static void frog(String key, Object... values) {
        Logging.log(key, values);
    }
    public static void frogWithDelay(String key, double delaySeconds, Object... values) {
        Logging.log(key, delaySeconds, values);
    }
}

