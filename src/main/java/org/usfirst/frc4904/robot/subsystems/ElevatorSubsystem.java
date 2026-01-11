package org.usfirst.frc4904.robot.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.*;
import org.usfirst.frc4904.robot.RobotMap.Component;
import org.usfirst.frc4904.standard.commands.NoOp;
import org.usfirst.frc4904.standard.custom.CustomEncoder;
import org.usfirst.frc4904.standard.custom.motioncontrollers.ezMotion.FeedForward;
import org.usfirst.frc4904.standard.custom.motorcontrollers.SmartMotorController;

import java.util.HashMap;
import java.util.function.DoubleSupplier;

public class ElevatorSubsystem extends PIDMotorSubsystem {

    // TODO TUNING: elevator PID
    public static final double kS = 1, kV = 2, kA = 0.4, kG = 0.2;

    public static final double kP = 6, kI = 0, kD = 0;

    public static final double MAX_VEL = 8;
    public static final double MAX_ACCEL = MAX_VEL * 4; // accelerate to max speed in 1/4 of a second

    public static final double MIN_HEIGHT = 0;
    public static final double MAX_HEIGHT = 12.5; //12.5 normal, 6 w/o 3rd stage

    public final ElevatorFeedforward feedforward;
    public final CustomEncoder encoder;

    // make sure that all values defined in this enum are added to the 'positions' map in the constructor
    public enum Position {
        INTAKE,
        L2,
        L3
    }

    public static HashMap<Position, Double> positions = new HashMap<>();

    public ElevatorSubsystem(SmartMotorController motor1, SmartMotorController motor2, CustomEncoder encoder) {
        super(
            new SmartMotorController[] { motor1, motor2 },
            7
        );
        this.feedforward = new ElevatorFeedforward(kS, kG, kV, kA);
        this.encoder = encoder;

        positions.put(Position.INTAKE, 0.0);

        // TODO IMPORTANT: tune
        positions.put(Position.L2, 6.1);
        positions.put(Position.L3, 9.3);

        for (var pos : Position.values()) {
            if (positions.get(pos) == null) {
                System.err.println(
                    "ElevatorSubsystem.Position." +
                    pos.name() +
                    " is not defined in 'positions' map"
                );
            }
        }
    }

    /**
     * @return The current height of the elevator in Magical Encoder Units™
     */
    public double getPosition() {
        return encoder.get();
    }

    public boolean atBottom() {
        return getPosition() < 0.1;
    }

    /** Intake at the current elevator position */
    public Command c_intake() {
        return new SequentialCommandGroup(
            Component.ramp.c_forward().withTimeout(0.4),
            new ParallelDeadlineGroup(
                new WaitCommand(0.32),
                Component.ramp.c_forward(true),
                Component.outtake.c_forward(true)
            )
        );
    }

    /** Outtake at the current elevator position */
    public Command c_outtake() {
        return Component.outtake.c_forward(true).withTimeout(0.5);
    }

    /** Outtake at the current elevator position */
    public Command c_rampOuttake() {
        return new ParallelDeadlineGroup(
            new WaitCommand(1),
            Component.outtake.c_backward(true),
            Component.ramp.c_backward(true)
        );
    }

    /** Go to the specified position and then outtake */
    public Command c_outtakeAtPosition(Position pos) {
        return new SequentialCommandGroup(
            c_gotoPosition(pos).withTimeout(3),
            new ParallelDeadlineGroup(
                c_outtake(),
                c_controlVelocity(() -> 0)
            )
        );
    }

    @Override
    protected PIDController getPID() {
        return new PIDController(kP, kI, kD);
    }

    @Override
    protected FeedForward getFF() {
        return (double pos, double vel) -> feedforward.calculate(vel);
    }

    @Override
    protected TrapezoidProfile getProfile() {
        return new TrapezoidProfile(
            new TrapezoidProfile.Constraints(MAX_VEL, MAX_ACCEL)
        );
    }

    public Command c_controlVelocity(DoubleSupplier metersPerSecDealer) {
        return run(() -> {
            var ff = feedforward.calculate(metersPerSecDealer.getAsDouble());
            setVoltage(ff);
        });
    }

    public Command c_gotoPosition(Position pos) {
        Double height = positions.get(pos);

        if (height == null) {
            System.err.println("Tried to go to elevator setpoint that does not exist: " + pos.toString());
            return new NoOp();
        }

        return c_gotoPosition(height);
    }

    @Override
    public void setVoltage(double voltage) {
        setVoltage(voltage, false);
    }

    public void setVoltage(double voltage, boolean bypassSoftwareStop) {
        if (
            !bypassSoftwareStop && (
                (getPosition() >= MAX_HEIGHT && voltage > 0) ||
                (getPosition() <= MIN_HEIGHT && voltage < 0)
            )
        ) {
            voltage = 0;
        }
        super.setVoltage(voltage);
    }
}
