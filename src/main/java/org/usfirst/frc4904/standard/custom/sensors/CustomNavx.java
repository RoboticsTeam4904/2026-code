package org.usfirst.frc4904.standard.custom.sensors;

import com.studica.frc.Navx;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

import java.util.Arrays;

import static edu.wpi.first.units.Units.Fahrenheit;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

/**
 * NavX should be mounted to robot according to WPILib coordinates:
 * +X is forward, +Y is left, +Z is up
 */
public class CustomNavx {

    // units begone
    protected final Navx navx;

    /**
     * Creates a new NavX
     *
     * @param id the CAN id of the navx. For whatever reason, this must be 0. don't ask why.
     */
    public CustomNavx(int id) {
        navx = new Navx(id);
    }

    /** @return current angular velocity around axes (in theory {@code {x, y, z}}/{@code {roll, pitch, yaw}}), in rotations/sec */
    public double[] getAngularVel() {
        // TODO test negation
        return Arrays.stream(navx.getAngularVel())
                     .mapToDouble(vel -> vel.in(RotationsPerSecond))
                     .toArray();
    }

    /** @return current acceleration along axes {@code {x, y, z}}/{@code {forward, left, up}}, in meters/sec² */
    public double[] getLinearAccel() {
        // TODO test axes/negation
        return Arrays.stream(navx.getLinearAccel())
                     .mapToDouble(vel -> vel.in(MetersPerSecondPerSecond))
                     .toArray();
    }

    /** @return current yaw in rotations; positive = counterclockwise */
    public double getYaw() {
        return navx.getYaw().in(Rotations);
    }
    /** @return current pitch in rotations; positive = front up */
    public double getPitch() {
        return -navx.getRoll().in(Rotations);
    }
    /** @return current roll in rotations; positive = left side up, right side down */
    public double getRoll() {
        return navx.getPitch().in(Rotations);
    }

    /** @return current yaw as a {@link Rotation2d} */
    public Rotation2d getRotation2d() {
        // i don't trust the navx getRotation2d() to be in the right units/direction/etc.
        return Rotation2d.fromRotations(getYaw());
    }
    /** @return current rotation as a {@link Rotation3d} */
    public Rotation3d getRotation3d() {
        // return navx.getRotation3d();
        // inherit negation from custom methods
        return new Rotation3d(getRoll(), getPitch(), getYaw());
    }

    /** Set the zero position to the current yaw */
    public void zeroYaw() {
        navx.resetYaw();
    }

    /** Get current gyroscope chip temperature in freedom units (°F) */
    public double getGyroTemperature() {
        return navx.getTemperature().in(Fahrenheit);
    }

    public int selfTest(int[] test) {
        return navx.selfTest(test);
    }

}
