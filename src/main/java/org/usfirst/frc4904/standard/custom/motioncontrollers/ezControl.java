package org.usfirst.frc4904.standard.custom.motioncontrollers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import org.usfirst.frc4904.standard.custom.motioncontrollers.ezMotion.FeedForward;

public class ezControl {

    public final PIDController pid;
    public final FeedForward ff;

    public ezControl(double kP, double kI, double kD) {
        this(new PIDController(kP, kI, kD), FeedForward.NOOP);
    }

    public ezControl(double kP, double kI, double kD, FeedForward ff) {
        this(new PIDController(kP, kI, kD), ff);
    }

    public ezControl(PIDController pid) {
        this(pid, FeedForward.NOOP);
    }

    public ezControl(PIDController pid, FeedForward ff) {
        this.pid = pid;
        this.ff = ff;
    }

    public void reset() {
        pid.reset();
    }

    public double calculate(double current, TrapezoidProfile.State setpoint) {
        double pidOut = pid.calculate(current, setpoint.position);
        double ffOut = ff.calculate(setpoint.position, setpoint.velocity);
        return pidOut + ffOut;
    }
}
