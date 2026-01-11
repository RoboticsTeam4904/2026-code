package org.usfirst.frc4904.standard.custom;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

/**
 * Converts a DutyCycleEncoder into encoding a linear position
 * by storing the number of full revolutions since initialization.
 */
public class LinearDutyCycleEncoder {
    public final DutyCycleEncoder encoder;

    private double resetOffset = 0;

    public LinearDutyCycleEncoder(int channel) {
        encoder = new DutyCycleEncoder(channel);
        reset();
    }

    public void reset() {
        revolutions = 0;
        resetOffset = encoder.get();
    }

    int revolutions = 0;
    Double lastReading = null;

    public double get() {
        double reading = encoder.get();

        if (lastReading == null) {
            lastReading = reading;
            return reading;
        }

        if (Math.abs(reading - lastReading) >= 0.5) {
            if (reading < lastReading) {
                revolutions++;
            } else {
                revolutions--;
            }
        }

        lastReading = reading;

        return reading + revolutions - resetOffset;
    }
}
