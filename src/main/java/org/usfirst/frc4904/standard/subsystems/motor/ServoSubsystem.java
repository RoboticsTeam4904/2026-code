package org.usfirst.frc4904.standard.subsystems.motor;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ServoSubsystem extends SubsystemBase {

	protected final Servo[] servos;
	protected boolean inverted;
	protected double lastPosition;

	// Constants from wpilib's Servo.java.
	protected static final double MIN_ANGLE = 0;
	protected static final double MAX_ANGLE = 180;
	protected static final double ANGLE_RANGE = MAX_ANGLE - MIN_ANGLE;

	/**
	 * A class that wraps around a variable number of Servo objects to give them
	 * Subsystem functionality.
	 *
	 * @param inverted Inverts the direction of all of the Servos. This does not
	 *                   override the individual inversions of the servos.
	 * @param servos     The Servos in this subsystem. Can be a single Servo or multiple.
	 */
	public ServoSubsystem(boolean inverted, Servo... servos) {
		super();

		this.servos = servos;
		this.inverted = inverted;
		setPosition(0);
	}

	/**
	 * A class that wraps around a variable number of Servo objects to give them
	 * Subsystem functionality.
	 *
	 * @param servos The Servos in this subsystem. Can be a single Servo or multiple.
	 */
	public ServoSubsystem(Servo... servos) {
		this(false, servos);
	}

	/**
	 * Get the servo position.
	 *
	 * Servo positions range from 0.0 to 1.0 corresponding to the range of full left
	 * to full right.
	 *
	 * @return Position from 0.0 to 1.0.
	 */
	public double getPosition() {
		return lastPosition;
	}

	/**
	 * Get the servo angle.
	 *
	 * @return The angle in degrees to which the servo is set.
	 */
	public double getAngle() {
		return positionToAngle(getPosition());
	}

	/**
	 * Set the servo position.
	 *
	 * Servo positions range from 0.0 to 1.0 corresponding to the range of full left
	 * to full right.
	 *
	 * @param position Position from 0.0 to 1.0.
	 */
	public void setPosition(double position) {
		if (inverted) {
			position = invertPosition(position);
		}
		for (Servo servo : servos) {
			servo.set(position);
		}
		lastPosition = position;
	}

	/**
	 * Set the servo angle.
	 *
	 * @return The angle in degrees to which the servo is set.
	 */
	public void setAngle(double degrees) {
		if (inverted) {
			degrees = invertAngle(degrees);
		}
		for (Servo servo : servos) {
			servo.setAngle(degrees);
		}
	}

	/**
	 * Get whether this entire servo is inverted.
	 *
	 * @return isInverted The state of inversion, true is inverted.
	 */
	public boolean getInverted() {
		return inverted;
	}

	/**
	 * Set whether this entire servo is inverted. Note that this will also convert
	 * the values returned by {@link #getPosition()} and {@link #getAngle() getAngle()}
	 * to the new coordinate system, so anything continuously reading these methods
	 * will see a discontinuity.
	 * 
	 * @param inverted The desired state of inversion, true is inverted.
	 */
	public void setInverted(boolean inverted) {
		if (inverted != this.inverted) {
			lastPosition = invertPosition(lastPosition);
		}
		this.inverted = inverted;
	}

	// Internal helper functions

	/**
	 * Convert a servo set value to degrees.
	 * 
	 * @param position servo set value to convert to degrees. Should be in the range [0, 1].
	 * @return the value converted to degrees
	 */
	protected static double positionToAngle(double position) {
		return position * ANGLE_RANGE + MIN_ANGLE;
	}

	/**
	 * Convert a degree to a servo set value.
	 * 
	 * @param degrees the servo degree to convert to a servo set
	 * @return a servo set value in the range [0, 1] (as long as the input degree
	 *         was in the servo's range)
	 */
	protected static double angleToPosition(double degrees) {
		return (degrees - MIN_ANGLE) / ANGLE_RANGE;
	}

	/**
	 * Invert the given value (0 becomes 1, 1 becomes 0)
	 * 
	 * @param position The value to invert. Should be in the range [0, 1]
	 * @return the inverted value
	 */
	protected static double invertPosition(double position) {
		return 1 - position;
	}

	/**
	 * Invert the given degree ({@link #MIN_ANGLE} becomes {@link #MAX_ANGLE},
	 * {@link #MAX_ANGLE} becomes {@link #MIN_ANGLE})
	 * 
	 * @param degrees The degree to invert
	 * @return the inverted degree
	 */
	protected static double invertAngle(double degrees) {
		return MAX_ANGLE - degrees + MIN_ANGLE;
	}
}
