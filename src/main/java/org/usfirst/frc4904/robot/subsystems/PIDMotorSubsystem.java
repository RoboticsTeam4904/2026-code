package org.usfirst.frc4904.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import org.usfirst.frc4904.standard.custom.motioncontrollers.ezMotion;
import org.usfirst.frc4904.standard.custom.motioncontrollers.ezMotion.FeedForward;
import org.usfirst.frc4904.standard.custom.motioncontrollers.ezMotion.Setpoint;
import org.usfirst.frc4904.standard.custom.motorcontrollers.SmartMotorController;

import java.util.Set;
import java.util.function.DoubleFunction;

public abstract class PIDMotorSubsystem extends MotorSubsystem {
    private final PIDController pid;
    private final FeedForward ff;
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

        pid = getPID();
        ff = getFF();
        profile = getProfile();
    }

    // the following methods are only called once and should be pure
    protected abstract PIDController getPID();
    // you probably want to either override BOTH or NEITHER of the following methods
    protected FeedForward getFF() { return FeedForward.NOOP; }
    protected TrapezoidProfile getProfile() { return null; }

    // get the current position (presumably from an encoder)
    public abstract double getPosition();
    // overriding this could lead to more accurate TrapezoidProfile calculation
    public double getVelocity() { return 0; }

    public Command c_gotoPosition(double position) {
        // don't calculate current pos until command starts
        return new DeferredCommand(() -> c_gotoPosition0(position), Set.of(this));
    }

    private Command c_gotoPosition0(double position) {
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

        // TODO stop when at setpoint
        return new ezMotion(
            pid,
            ff,
            this::getPosition,
            this::setVoltage,
            (double elapsed) -> profile != null
                ? new Setpoint(profile.calculate(elapsed, current, goal))
                : setpoint,
            this
        ).finallyDo(this::stop);
    }
}
