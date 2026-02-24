package org.usfirst.frc4904.standard.custom.sensors;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.util.Units;

import static edu.wpi.first.units.Units.*;

public class CustomPigeon implements IMU {

    protected final Pigeon2 pigeon;

    public CustomPigeon(int deviceId) {
        pigeon = new Pigeon2(deviceId);
    }

    // TODO test axes and negation
    @Override
    public double getYaw() {
        return IMU.wrapAnglePositive(pigeon.getYaw().getValue().in(Rotations));
    }
    @Override
    public double getPitch() {
        return IMU.wrapAngleHalf(pigeon.getPitch().getValue().in(Rotations));
    }
    @Override
    public double getRoll() {
        return IMU.wrapAngleHalf(pigeon.getRoll().getValue().in(Rotations));
    }

    @Override
    public void zeroYaw(double offset) {
        pigeon.setYaw(Units.rotationsToDegrees(offset));
    }

    @Override
    public double getTemperature() {
        return pigeon.getTemperature().getValue().in(Fahrenheit);
    }

}
