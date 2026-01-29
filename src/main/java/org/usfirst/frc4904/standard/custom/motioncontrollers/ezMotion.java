package org.usfirst.frc4904.standard.custom.motioncontrollers;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class ezMotion extends Command {

    public final ezControl control;

    private final DoubleSupplier getCurrent;
    private final DoubleConsumer processValue;
    private final Supplier<? extends SetpointSupplier> setpointDealerDealer;

    private SetpointSupplier setpointDealer;
    private double initialTimestamp;

    public ezMotion(
        ezControl control,
        DoubleSupplier getCurrent,
        DoubleConsumer processValue,
        Supplier<? extends SetpointSupplier> setpointDealerDealer,
        Subsystem... requirements
    ) {
        this.control = control;
        this.getCurrent = getCurrent;
        this.processValue = processValue;
        this.setpointDealerDealer = setpointDealerDealer;

        addRequirements(requirements);
    }

    public ezMotion(
        ezControl control,
        DoubleSupplier getCurrent,
        DoubleConsumer processValue,
        SetpointSupplier setpointDealer,
        Subsystem... requirements
    ) {
        this(control, getCurrent, processValue, () -> setpointDealer, requirements);
    }

    public ezMotion(
        ezControl control,
        DoubleSupplier getCurrent,
        DoubleConsumer processValue,
        TrapezoidProfile.State goal,
        TrapezoidProfile.Constraints constraints,
        Subsystem... requirements
    ) {
        this(
            control,
            getCurrent,
            processValue,
            () -> {
                var profile = new TrapezoidProfile(constraints);

                var start = new TrapezoidProfile.State(getCurrent.getAsDouble(), 0);

                return (elapsed) -> profile.calculate(elapsed, start, goal);
            },
            requirements
        );
    }

    public double getElapsedTime() {
        return Timer.getFPGATimestamp() - initialTimestamp;
    }

    @Override
    public void initialize() {
        initialTimestamp = Timer.getFPGATimestamp();
        setpointDealer = setpointDealerDealer.get();
    }

    @Override
    public void execute() {
        TrapezoidProfile.State setpoint = setpointDealer.get(getElapsedTime());
        double controlEffort = control.calculate(getCurrent.getAsDouble(), setpoint);
        processValue.accept(controlEffort);
    }

    @FunctionalInterface
    public interface SetpointSupplier {
        TrapezoidProfile.State get(double elapsed);
    }

    @FunctionalInterface
    public interface FeedForward {
        FeedForward NOOP = (pos, vel) -> 0;

        double calculate(double position, double velocity);
    }
}
