// TODO untested

package org.usfirst.frc4904.standard.custom;

import edu.wpi.first.wpilibj.Timer;
import org.usfirst.frc4904.standard.Util;

import java.util.function.DoubleSupplier;

public class VelocityEstimator {

    private static final double MIN_DT = 0.01; // minimum time (sec) between samples to not ignore second sample
    private static final double MAX_DT = 1; // maximum time (sec) between samples before resetting

    private final DoubleSupplier posSupplier;
    private final double maxAccel;
    private final double accuracy;

    private boolean hasEstimate;
    private double lastPos;
    private double lastTime;

    // estimated velocity = this.velocity ± this.error
    private double velocity;
    private double error;

    /**
     * Estimate the velocity of something based on position data. You probably want to call {@code getVelocity()} or {@code periodic()}
     * on every scheduler iteration whenever there is a possibility that the code will need a velocity estimate soon.
     *
     * @param posSupplier Supplier of position data.
     * @param maxAccel Maximum acceleration of the thing being measured. Used to calculate error.
     * @param accuracy Strange value that roughly corresponds to the accuracy of the position measurements.
     *                 Skews the weighted average between the current velocity estimate and the last.
     *                 1.0 ignores the previous estimate entirely, and 0.5-0.8 is probably reasonable for most things.
     *                 0.0 doesn't completely ignore the current estimate, as the time since the previous sample is also factored in.
     */
    public VelocityEstimator(DoubleSupplier posSupplier, double maxAccel, double accuracy) {
        if (accuracy < 0 || accuracy > 1) {
            throw new IllegalArgumentException("VelocityEstimator accuracy must be between 0 and 1");
        }

        this.posSupplier = posSupplier;
        this.maxAccel = maxAccel;
        this.accuracy = accuracy;

        reset();
    }

    public void reset() {
        lastPos = posSupplier.getAsDouble();
        lastTime = Timer.getFPGATimestamp();
        hasEstimate = false;
    }

    // call this frequently if you may need velocity estimates in the near future to keep estimates accurate
    public void periodic() {
        double now = Timer.getFPGATimestamp();
        double dt = now - lastTime;

        if (dt < MIN_DT) return;
        if (dt > MAX_DT) {
            reset();
            return;
        }

        double pos = posSupplier.getAsDouble();
        double dx = pos - lastPos;

        double newVelocity = dx / dt;
        // rough estimate
        // according to my (possibly incorrect) calculations, worst case is 1.5 * maxAccel * dt
        double newError = maxAccel * dt;

        if (hasEstimate) {
            // weighted average with previous measurement
            // lean more towards new measurement the closer we are to MAX_DT
            double oldWeight = (1 - accuracy) * (1 - dt / MAX_DT);
            newVelocity = velocity * oldWeight + newVelocity * (1 - oldWeight);
        }
        // ensure new velocity is within a vaguely reasonable margin of the old one
        double margin = error + newError;
        newVelocity = Util.clamp(newVelocity, velocity - margin, velocity + margin);

        velocity = newVelocity;
        error = newError;
        lastPos = pos;
        lastTime = now;
        hasEstimate = true;
    }

    public double getVelocity() {
        periodic();
        return hasEstimate ? velocity : 0;
    }

    public double getError() {
        periodic();
        return hasEstimate ? error : Double.POSITIVE_INFINITY;
    }

}
