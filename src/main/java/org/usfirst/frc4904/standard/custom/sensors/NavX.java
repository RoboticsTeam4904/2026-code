// WAS PID SOURCE
package org.usfirst.frc4904.standard.custom.sensors;

import static edu.wpi.first.units.Units.Degrees;

import com.studica.frc.Navx;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

/**
 * Local NavX interface.
 */
public class NavX extends Navx {
    protected Angle lastYaw;
    protected Angle lastPitch;
    protected Angle lastRoll;
    protected double lastYawRate;
    public static int outputRate;
    protected int getYawCalls;
    protected static final double MAX_DEGREES_PER_TICK = 90.0;
    protected static final double MAX_DEGREES_PER_SECOND_PER_TICK = 180;

    public NavX(int id, int rate) {
        super(id, rate);
        super.resetYaw();
        lastYaw = Degrees.of(0);
        lastPitch = Degrees.of(0);
        lastRoll = Degrees.of(0);
        outputRate = rate;
        getYawCalls = 0;
    }

    public double getRate() {
        double rate = outputRate;
        if (Math.abs(rate - lastYawRate) > NavX.MAX_DEGREES_PER_SECOND_PER_TICK) {
            return lastYawRate;
        }
        lastYawRate = rate;
        return rate;
    }

    /**
     * Returns an always positive yaw. Ignores anomalous values
     */
    public Angle getSafeYaw() {
        Angle yaw = super.getYaw();
        // SmartDashboard.putNumber("navx_yaw", yaw);
        // SmartDashboard.putNumber("navx_last_yaw", lastYaw);
        if ((Math.abs(yaw.in(Degrees) - lastYaw.in(Degrees)) > NavX.MAX_DEGREES_PER_TICK)
                && (Math.abs(Math.abs(yaw.in(Degrees) - lastYaw.in(Degrees)) - 360) > NavX.MAX_DEGREES_PER_TICK)) { // Smoothing
            return lastYaw;
        }
        lastYaw = yaw;
        return yaw;
    }

    @Override
    public Angle getYaw() {
        getYawCalls += 1;
        return super.getYaw();
    }

    /**
     * Returns an always positive pitch
     */
    @Override
    public Angle getPitch() {
        Angle pitch = super.getPitch();
        if (Math.abs(pitch.in(Degrees) - lastPitch.in(Degrees)) > NavX.MAX_DEGREES_PER_TICK) {
            return lastPitch;
        }
        if (pitch.in(Degrees) < 0) {
            lastPitch = Degrees.of(360 + pitch.in(Degrees));
            return Degrees.of(360 + pitch.in(Degrees));
        } else {
            lastPitch = pitch;
            return pitch;
        }
    }

    /**
     * Returns an always positive roll
     */
    @Override
    public Angle getRoll() {
        Angle roll = super.getRoll();
        if (Math.abs(roll.in(Degrees) - lastRoll.in(Degrees)) > NavX.MAX_DEGREES_PER_TICK) {
            return lastRoll;
        }
        if (roll.in(Degrees) < 0) {
            lastRoll = Degrees.of(360 + roll.in(Degrees));
            return Degrees.of(360 + roll.in(Degrees));
        } else {
            lastRoll = roll;
            return roll;
        }
    }

    public void zeroYaw() {
        super.resetYaw();
        lastYaw = Degrees.of(0);
    }
}
