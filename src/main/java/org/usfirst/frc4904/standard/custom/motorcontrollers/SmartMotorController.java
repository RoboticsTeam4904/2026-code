package org.usfirst.frc4904.standard.custom.motorcontrollers;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

/**
 * Represents a "smart" motor controller, like the TalonFX or SparkMax.
 * <p>
 * These should support brake mode, follow mode, limit switches, and various closed-loop control modes.
 */
public interface SmartMotorController extends MotorController {
	boolean isFwdLimitSwitchPressed() throws IllegalAccessException;
	boolean isRevLimitSwitchPressed() throws IllegalAccessException;

	void setMotorBrake(boolean brake);

	/**
	 * Get the current rotation of the motor.
	 * Affected by {@code setMotorMechanismRatio()} and {@code setContinuous()}/{@code setContinuousRange()}.
	 */
	double getRotation();

	/**
	 * Set the number of motor rotations per mechanism rotation.
	 * Used for {@code getPosition()}, {@code holdPosition()}, and arm feedforward calculations.
	 */
	SmartMotorController setMotorMechanismRatio(double ratio);

	/**
	 * Get the ratio between motor rotations and mechanism rotations set by {@code setMotorMechanismRatio()}.
	 */
	double getMotorMechanismRatio();

	/**
	 * Sets the current rotation position as the "zero" position.
	 * Equivalent to {@code setMechanismRotationOffset(currentRotation - currentMechanismRotationOffset)}.
	 */
	default void zeroMechanismRotationOffset() {
		setMechanismRotationOffset(getRotation() - getMechanismRotationOffset());
		// not chainable since this isn't a config method
	}

	/**
	 * Set the offset between motor rotations and mechanism rotations.
	 * DO NOT USE with linear mechanisms or elevator feedforward, only with rotational mechanisms.
	 * @param offset Mechanism offset, continuous between 0-1.
	 */
	SmartMotorController setMechanismRotationOffset(double offset);

	/**
	 * Get the offset between motor rotations and mechanism rotations set by {@code setMechanismRotationOffset()}.
	 */
	double getMechanismRotationOffset();

	SmartMotorController setPID(double p, double i, double d);

	/**
	 * For static gravity gain.
	 * @see ElevatorFeedforward#ElevatorFeedforward(double, double, double, double)
	 */
	SmartMotorController setElevFF(double kS, double kG, double kV, double kA);
	/**
	 * For gravity gain depending on the rotation of the mechanism.
	 * @param kG Gravity constant that will be multiplied by cos(mechanismRotation).
	 * @see ArmFeedforward#ArmFeedforward(double, double, double, double)
	 */
	SmartMotorController setArmFF(double kS, double kG, double kV, double kA);

	/**
	 * Enable or disable continuous input (0 = 1 / 0deg = 360deg).
	 */
	default SmartMotorController setContinuous(boolean enabled) {
		return setContinuousRange(enabled ? 1 : 0);
	}
	/**
	 * Set range for continuous motion.
	 * @param range Distance of one full rotation. Analogous to {@code PIDController.enableContinuousInput(0, range)}.
	 *              If range = 0.5, then 0 = 0.5 = 1, or 0deg = 180deg = 360deg.
	 *              Use {@code setContinuous(false)} or pass 0 to disable continuous input.
	 */
	SmartMotorController setContinuousRange(double range);

	default boolean isContinuous() {
		return getContinuousRange() != 0;
	}
	/**
	 * Get the current continuous range.
	 * @return Current range or 0 if not continuous.
	 */
	double getContinuousRange();

	/**
	 * Holds a position using this slot's configured PID and {@code kG}.
	 * Runs until one of {@code set()}, {@code setVoltage()}, {@code stopMotor()}, etc. is called on the motor.
	 * @param pos Position in rotations.
	 */
	default void holdPosition(double pos) {
		holdPosition(pos, 0);
	}
	/**
	 * Holds a position using this slot's configured PID and {@code kG}.
	 * Runs until one of {@code set()}, {@code setVoltage()}, {@code stopMotor()}, etc. is called on the motor.
	 * @param pos Position in rotations.
	 * @param addedVoltage Extra voltage to add after PID/FF calculations.
	 */
	void holdPosition(double pos, double addedVoltage);

	/**
	 * Holds a velocity using this slot's configured PID and FF.
	 * Runs until one of {@code set()}, {@code setVoltage()}, {@code stopMotor()}, etc. is called on the motor.
	 * @param vel Velocity in rotations/sec.
	 */
	default void holdVelocity(double vel) {
		holdVelocity(vel, 0);
	}
	/**
	 * Holds a velocity using this slot's configured PID and FF.
	 * Runs until one of {@code set()}, {@code setVoltage()}, {@code stopMotor()}, etc. is called on the motor.
	 * @param vel Velocity in rotations/sec.
	 * @param addedVoltage Extra voltage to add after PID/FF calculations.
	 */
	void holdVelocity(double vel, double addedVoltage);

}
