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
    public final PIDController pid;
    public final FeedForward ff;

    private final DoubleSupplier getCurrent;
    private final DoubleConsumer processValue;
    private final DoubleFunction<Setpoint> setpointDealer;

    public double startTime;

    public ezMotion(
        PIDController pid,
        FeedForward ff,
        DoubleSupplier getCurrent,
        DoubleConsumer processValue,
        DoubleFunction<Setpoint> setpointDealer,
        Subsystem... requirements
    ) {
        this.pid = pid;
        this.ff = ff;
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
    }

    @Override
    public void execute() {
        double current = getCurrent.getAsDouble();
        Setpoint setpoint = setpointDealer.apply(getElapsedTime());

        double calculated = pid.calculate(current, setpoint.pos) + ff.calculate(setpoint.pos, setpoint.vel);
        processValue.accept(calculated);
    }

    @FunctionalInterface
    public interface FeedForward {
        FeedForward NOOP = (double pos, double vel) -> 0;

        double calculate(double pos, double vel);
    }

    public record Setpoint(double pos, double vel) {
        public Setpoint(TrapezoidProfile.State state) {
            this(state.position, state.velocity);
        }
    }
}