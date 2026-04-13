package lib.custom.sensors;

import lib.custom.Nameable;

/**
 * A sensor that provides distance values (of type `double`).
 */
public interface DistanceSensor extends Nameable {
	double getDistance();

	double getDistanceSafely() throws InvalidSensorException;
}
