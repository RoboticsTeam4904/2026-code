package org.usfirst.frc4904.standard.custom.sensors;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Preferences;

public class CustomDutyCycleEncoder extends DutyCycleEncoder {

    private final String key;

    private double resetOffset;

    public CustomDutyCycleEncoder(int channel) {
        super(channel);

        key = "zeros/" + channel;

        Preferences.initDouble(key, 0);
        resetOffset = Preferences.getDouble(key, 0);
    }

    public void reset() {
        resetOffset = super.get();
        Preferences.setDouble(key, resetOffset);
    }

    @Override
    public double get() {
        double value = super.get() - resetOffset;
        return value < 0 ? value + 1 : value;
    }
}
