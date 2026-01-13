package org.usfirst.frc4904.robot.subsystems;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import org.usfirst.frc4904.standard.custom.motioncontrollers.ezMotion;
import org.usfirst.frc4904.standard.custom.motioncontrollers.ezMotion.Setpoint;
import org.usfirst.frc4904.standard.custom.motioncontrollers.ezMotion.ezControl;
import org.usfirst.frc4904.standard.custom.motorcontrollers.SmartMotorController;

import java.util.function.DoubleSupplier;

public abstract class PIDMotorSubsystem extends MotorSubsystem {
    private final ezControl control;
    private final TrapezoidProfile profile;

    /**
     * @see MotorSubsystem#MotorSubsystem(SmartMotorController, double)
     */
    public PIDMotorSubsystem(SmartMotorController motor, double voltage) {
        this(new SmartMotorController[] { motor }, voltage, voltage);
    }

    /**
     * @see MotorSubsystem#MotorSubsystem(SmartMotorController, double, double)
     */
    public PIDMotorSubsystem(SmartMotorController motor, double forwardVoltage, double backwardVoltage) {
        this(new SmartMotorController[] { motor }, forwardVoltage, backwardVoltage);
    }

    /**
     * @see MotorSubsystem#MotorSubsystem(SmartMotorController[], double)
     */
    public PIDMotorSubsystem(SmartMotorController[] motors, double voltage) {
        this(motors, voltage, voltage);
    }

    /**
     * @see MotorSubsystem#MotorSubsystem(SmartMotorController[], double, double)
     */
    public PIDMotorSubsystem(SmartMotorController[] motors, double forwardVoltage, double backwardVoltage) {
        super(motors, forwardVoltage, backwardVoltage);

        control = getControl();
        profile = new TrapezoidProfile(getConstraints());
    }

    // the following methods are only called once and should be pure
    protected abstract ezControl getControl();
    // should only be overridden if the ezControl has a feedforward
    protected TrapezoidProfile.Constraints getConstraints() { return null; }

    // get the current position (presumably from an encoder)
    public abstract double getPosition();
    // overriding this could lead to more accurate TrapezoidProfile calculation
    public double getVelocity() { return 0; }

    // only has an effect if the ezControl has a feedforward
    public Command c_controlVelocity(DoubleSupplier velocityDealer) {
        return run(() -> {
            Setpoint sp = new Setpoint(getPosition(), velocityDealer.getAsDouble());
            setVoltage(control.ff().calculate(sp));
        });
    }

    public Command c_holdCurrentPosition() {
        return c_gotoPosition(getPosition());
    }

    public Command c_gotoPosition(double position) {
        return c_gotoPosition(position, false);
    }

    public Command c_gotoPosition(double position, boolean endOnArrival) {
        // don't calculate current position until command starts
        return defer(() -> c_gotoPosition0(position, endOnArrival));
    }

    private Command c_gotoPosition0(double position, boolean endOnArrival) {

        TrapezoidProfile.State current, goal;
        Setpoint setpoint;
        if (profile != null) {
            current = new TrapezoidProfile.State(getPosition(), getVelocity());
            goal = new TrapezoidProfile.State(position, 0);
            setpoint = null;
        } else {
            setpoint = new Setpoint(position, 0);
            current = null;
            goal = null;
        }

        return new ezMotion(
            control,
            this::getPosition,
            this::setVoltage,
            (double elapsed) -> profile != null
                ? new Setpoint(profile.calculate(elapsed, current, goal))
                : setpoint,
            this
        ) {
            @Override
            public boolean isFinished() {
                if (!endOnArrival) return false;

                double error = Math.abs(position - getPosition());
                return error < control.pid().getErrorTolerance();
            }

            @Override
            public void end(boolean interrupted) {
                super.end(interrupted);
                if (!interrupted) stop();
            }
        };
    }
}
