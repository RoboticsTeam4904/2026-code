package org.usfirst.frc4904.robot.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc4904.standard.util.MathUtil;
import org.usfirst.frc4904.standard.custom.motorcontrollers.SmartMotorController;
import org.usfirst.frc4904.standard.custom.sensors.CustomDutyCycleEncoder;

public class SwerveModule {

    public final String name;

    private final DriveController drive;
    private final RotationController rotation;

    private double magnitude = 0;
    private double theta = 0;

    public SwerveModule(
        String name,
        SmartMotorController driveMotor,
        SmartMotorController rotMotor,
        CustomDutyCycleEncoder rotEncoder,
        Translation2d direction
    ) {
        this.name = name;

        // TODO remove (or maybe keep for comp?)
        if (driveMotor != null) {
            drive = new DriveController(driveMotor);
        } else {
            drive = null;
        }
        rotation = new RotationController(rotMotor, rotEncoder, direction);
    }

    public Translation2d rotToTranslation(double theta) {
        return rotation.toTranslation(theta);
    }

    public void zero() {
        rotation.zero();
    }

    public void moveTo(double magnitude, double theta) {
        this.magnitude = magnitude;
        if (magnitude > 0) this.theta = theta;
    }

    public void periodic() {
        // TODO run this faster than 50hz - run pid on motor
        boolean flip = rotation.rotateToward(theta);
        if (drive != null) drive.setMagnitude(flip ? -magnitude : magnitude);

        SmartDashboard.putNumber(name + " rotation: ", rotation.encoder.get());
    }
}

record DriveController(SmartMotorController motor) {
    public void setMagnitude(double magnitude) {
        motor.set(magnitude / SwerveConstants.LIN_SPEED);
    }
}

class RotationController {
    private static final double kP = 10, kI = 0, kD = 0;

    private static final double MAX_VOLTAGE = 4;

    final SmartMotorController motor;
    final CustomDutyCycleEncoder encoder;

    private final Translation2d direction;

    private final PIDController pid;

    public RotationController(
        SmartMotorController motor,
        CustomDutyCycleEncoder encoder,
        Translation2d direction
    ) {
        this.motor = motor;

        this.encoder = encoder;

        this.direction = direction.div(direction.getNorm());

        this.pid = new PIDController(kP * 1.5, kI, kD);
        // encoder readings are from 0-1 but opposite angles are equivalent
        // since we can just run the wheels backwards
        this.pid.enableContinuousInput(0, 0.5);
    }

    public Translation2d toTranslation(double theta) {
        return direction.times(theta);
    }

    public void zero() {
        encoder.reset();
    }

    private double getRotation() {
        return encoder.get();
    }

    private void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    /**
     * @return True if the wheel is currently more than halfway off the target
     *         and therefore should drive in the opposite direction.
     */
    public boolean rotateToward(double theta) {
        double current = getRotation();
        double voltage = -pid.calculate(current, theta);
        setVoltage(MathUtil.clamp(voltage, -MAX_VOLTAGE, MAX_VOLTAGE));

        double dist = Math.abs(theta - current);
        return dist > 0.25 && dist < 0.75;
    }
}
