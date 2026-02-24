package org.usfirst.frc4904.standard.custom.sensors;

import com.studica.frc.Navx;

import static edu.wpi.first.units.Units.Fahrenheit;
import static edu.wpi.first.units.Units.Rotations;

/**
 * NavX should be mounted to robot according to WPILib coordinates:
 * +X is forward, +Y is left, +Z is up
 */
public class CustomNavx implements IMU {

    protected final Navx navx;
    private double yawOffset;

    /**
     * Creates a new NavX
     *
     * @param id the CAN id of the navx. For whatever reason, this must be 0. Don't ask why.
     */
    public CustomNavx(int id) {
        navx = new Navx(id);
    }

    @Override
    public double getYaw() {
        return IMU.wrapAnglePositive(navx.getYaw().in(Rotations) + yawOffset);
    }
    @Override
    public double getPitch() {
        return -navx.getRoll().in(Rotations);
    }
    @Override
    public double getRoll() {
        return navx.getPitch().in(Rotations);
    }

    @Override
    public void zeroYaw(double offset) {
        yawOffset = offset;
        navx.resetYaw();
    }

    @Override
    public double getTemperature() {
        return navx.getTemperature().in(Fahrenheit);
    }

}
