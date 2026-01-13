package org.usfirst.frc4904.standard.custom.motioncontrollers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleFunction;
import java.util.function.DoubleSupplier;

public class ezMotion extends Command {
    public final ezControl control;

    private final DoubleSupplier getCurrent;
    private final DoubleConsumer processValue;
    private final DoubleFunction<Setpoint> setpointDealer;

    public double startTime;

    public ezMotion(
        ezControl control,
        DoubleSupplier getCurrent,
        DoubleConsumer processValue,
        Setpoint setpoint,
        Subsystem... requirements
    ) {
        this(control, getCurrent, processValue, (double elapsed) -> setpoint, requirements);
    }

    public ezMotion(
        ezControl control,
        DoubleSupplier getCurrent,
        DoubleConsumer processValue,
        DoubleFunction<Setpoint> setpointDealer,
        Subsystem... requirements
    ) {
        this.control = control;
        this.getCurrent = getCurrent;
        this.processValue = processValue;
        this.setpointDealer = setpointDealer;

        addRequirements(requirements);
    }

    public double getElapsedTime() {
        return Timer.getFPGATimestamp() - startTime;
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        control.pid().reset();
    }

    @Override
    public void execute() {
        double current = getCurrent.getAsDouble();
        Setpoint setpoint = setpointDealer.apply(getElapsedTime());
        processValue.accept(control.calculate(current, setpoint));
    }

    public record ezControl(PIDController pid, FeedForward ff) {
        public ezControl(PIDController pid) {
            this(pid, FeedForward.NOOP);
        }

        double calculate(double current, Setpoint sp) {
            return pid.calculate(current, sp.position) + ff.calculate(sp);
        }
    }

    @FunctionalInterface
    public interface FeedForward {
        FeedForward NOOP = (Setpoint sp) -> 0;

        double calculate(Setpoint setpoint);
    }

    // doesn't necessarily have to be position and velocity - can be any value and its derivative
    public record Setpoint(double position, double velocity) {
        public Setpoint(TrapezoidProfile.State state) {
            this(state.position, state.velocity);
        }
    }
}
