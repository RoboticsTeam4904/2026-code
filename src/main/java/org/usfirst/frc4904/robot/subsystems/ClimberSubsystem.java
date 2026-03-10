package org.usfirst.frc4904.robot.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

import org.usfirst.frc4904.standard.custom.motioncontrollers.ezControl;
import org.usfirst.frc4904.standard.custom.motioncontrollers.ezMotion;
import org.usfirst.frc4904.standard.custom.motorcontrollers.SmartMotorController;

import edu.wpi.first.wpilibj2.command.Command;
import org.usfirst.frc4904.standard.custom.sensors.LinearDutyCycleEncoder;
import org.usfirst.frc4904.standard.util.Logging;
import org.usfirst.frc4904.standard.util.Util;

public class ClimberSubsystem extends MotorSubsystem {

    // TODO change all these
    // public static final double kS = 0;
    // public static final double kV = 30;
    // public static final double kA = 0;
    // public static final double kG = 0;

    public static final double kP = 30;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double MAX_VOLTAGE = 6;

    // public static final double MAX_VEL = 0.3;
    // public static final double MAX_ACCEL = 10;

    private static final double MAX_HEIGHT = 2.352;
    private static final double MIN_HEIGHT = 0.623;
    
    private final LinearDutyCycleEncoder encoder;

    public ClimberSubsystem(SmartMotorController motor, LinearDutyCycleEncoder encoder) {
        super(motor, 3);

        this.encoder = encoder;
    }

    public Command c_up() {
        return c_forward(true);
    }

    public Command c_down() {
        return c_backward(true);
    }

    public double getHeight() {
        return encoder.get();
    }

    public Command c_gotoUp() {
        return c_gotoHeight(MAX_HEIGHT);
    }

    public Command c_gotoDown() {
        return c_gotoHeight(MIN_HEIGHT);
    }

    public Command c_gotoHeight(double height) {
        var pid = new PIDController(kP, kI, kD);

        return c_controlVoltage(() -> {
            double effort = pid.calculate(getHeight(), height);
            return Util.clamp(effort, -MAX_VOLTAGE, MAX_VOLTAGE);
        }, true);
    }

    @Override
    public void setVoltage(double voltage) {
        setVoltage(voltage, false);
    }

    public void setVoltage(double voltage, boolean bypassSoftwareStop) {
        if (
            !bypassSoftwareStop && (
                (this.getHeight() >= MAX_HEIGHT && voltage > 0) ||
                (this.getHeight() <= MIN_HEIGHT && voltage < 0)
            )
        ) {
            voltage = 0;
        }
        super.setVoltage(voltage);
    }
}
