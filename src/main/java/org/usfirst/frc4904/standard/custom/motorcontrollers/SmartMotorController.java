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

	void setBrakeOnNeutral();
	void setCoastOnNeutral();

	/**
	 * Implementing classes should ensure that consecutive calls with the same
	 * slot number return the same {@link SmartMotorConfigSlot} instance.
	 *
	 * @throws IllegalArgumentException when slot is out of range.
	 */
	SmartMotorConfigSlot configSlot(int slot);

	interface SmartMotorConfigSlot {

		SmartMotorConfigSlot setPID(double p, double i, double d);

		/**
		 * For static gravity gain.
		 * @see ElevatorFeedforward#ElevatorFeedforward(double, double, double, double)
		 */
		SmartMotorConfigSlot setElevFF(double kS, double kG, double kV, double kA);
		/**
		 * For gravity gain depending on the rotation of the mechanism.
		 * @param kG Gravity constant that will be multiplied by cos(mechanismRotation).
		 * @see ArmFeedforward#ArmFeedforward(double, double, double, double)
		 */
		default SmartMotorConfigSlot setArmFF(double kS, double kG, double kV, double kA) {
			return setArmFF(kS, kG, kV, kA, 0);
		}
		/**
		 * For gravity gain depending on the rotation of the mechanism.
		 * @param kG Gravity constant that will be multiplied by cos(mechanismRotation).
		 * @param kCosRatio Conversion factor from measured rotation (encoder units) to absolute rotation (rotations).
		 *                  Only supported by some motor controllers - will throw {@code IllegalArgumentException} if not supported and set to something other than 0.
		 * @see ArmFeedforward#ArmFeedforward(double, double, double, double)
		 */
		SmartMotorConfigSlot setArmFF(double kS, double kG, double kV, double kA, double kCosRatio);

		/**
		 * Enable or disable continuous input (0 = 1 / 0deg = 360deg).
		 */
		default SmartMotorConfigSlot continuous(boolean enabled) {
			return continuous(enabled ? 1 : 0);
		}
		/**
		 * Set range for continuous motion.
		 * @param range Distance of one full rotation.
		 *              If range = 0.5, then 0 = 0.5 = 1, or 0deg = 180deg = 360deg.
		 *              Set to false or 0 to disable continuous input.
		 */
		SmartMotorConfigSlot continuous(double range);

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
}
