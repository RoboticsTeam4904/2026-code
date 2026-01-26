package org.usfirst.frc4904.standard.custom.sensors;

import com.studica.frc.Navx;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

import java.util.Arrays;

import static edu.wpi.first.units.Units.*;

public class CustomNavx {

    // units begone
    protected final Navx navx;

    /** See {@link Navx#Navx(int)} */
    public CustomNavx(int id) {
        navx = new Navx(id);
    }

    /** @return current angular velocity around axes {@code {x, y, z}}/{@code {roll, pitch, yaw}}, in rotations/sec */
    public double[] getAngularVel() {
        return Arrays.stream(navx.getAngularVel())
                     .mapToDouble(vel -> vel.in(RotationsPerSecond))
                     .toArray();
    }

    /** @return current acceleration along axes {@code {x, y, z}}/{@code {forward, left, up}}, in meters/sec² */
    public double[] getLinearAccel() {
        return Arrays.stream(navx.getLinearAccel())
                     .mapToDouble(vel -> vel.in(MetersPerSecondPerSecond))
                     .toArray();
    }

    /** @return current yaw in rotations */
    public double getYaw() {
        return navx.getYaw().in(Rotations);
    }
    /** @return current pitch in rotations */
    public double getPitch() {
        return navx.getPitch().in(Rotations);
    }
    /** @return current roll in rotations */
    public double getRoll() {
        return navx.getRoll().in(Rotations);
    }

    /** @return current yaw as a {@link Rotation2d} */
    public Rotation2d getRotation2d() {
        return navx.getRotation2d();
    }
    /** @return current rotation as a {@link Rotation3d} */
    public Rotation3d getRotation3d() {
        return navx.getRotation3d();
    }

    /** Set the zero position to the current yaw */
    public void zeroYaw() {
        navx.resetYaw();
    }

    /** Get current gyroscope chip temperature in freedom units (°F) */
    public double getGyroTemperature() {
        return navx.getTemperature().in(Fahrenheit);
    }

}
