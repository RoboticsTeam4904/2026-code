package org.usfirst.frc4904.standard.silly;
import org.usfirst.frc4904.standard.util.Logging;

public final class Cheese {
    public static void tax(String key, Object... values) {
        Logging.log(key, values);
    }
    public static void taxWithDelay(String key, double delaySeconds, Object... values) {
        Logging.log(key, delaySeconds, values);
    }
}

