package org.usfirst.frc4904.robot.subsystems;

import org.usfirst.frc4904.standard.custom.motioncontrollers.ezControl;
import org.usfirst.frc4904.standard.custom.motioncontrollers.ezMotion;
import org.usfirst.frc4904.standard.custom.motorcontrollers.SmartMotorController;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeSubsystem extends MotorSubsystem {

    public static final double kS = 1;
    public static final double kV = 2;
    public static final double kA = 0.4;
    public static final double kG = 0.2;

    public static final double kP = 6;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double MAX_VEL = 8;
    public static final double MAX_ACCEL = MAX_VEL * 4; // accelerate to max speed in 1/4 of a second

    private final ArmFeedforward feedforward;
    private final DutyCycleEncoder encoder;

    public IntakeSubsystem(
        SmartMotorController intakeVerticalMotor,
        SmartMotorController intakeRollerMotor,
        DutyCycleEncoder intakeEncoder
    ) {
        super(
            new SmartMotorController[] { intakeRollerMotor },
            4
        );

        this.feedforward = new ArmFeedforward(kS, kG, kV, kA);
        this.encoder = intakeEncoder;
    }

    public double getAngle() {
        return encoder.get();
    }

    public Command c_intake() {
        return c_forward(true);
    }

    // TODO: actually find the angles
    public Command c_gotoAngle(double angle) {
        return defer(() -> getRawAngleCommand(angle));
    }

    private Command getRawAngleCommand(double angle) {
        ezControl controller = new ezControl(
            kP, kI, kD,
            (position, velocity) -> feedforward.calculate(getAngle(), velocity)
        );

        TrapezoidProfile profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(MAX_VEL, MAX_ACCEL)
        );

        var start = new TrapezoidProfile.State(getAngle(), 0);
        var goal = new TrapezoidProfile.State(angle, 0);

        return new ezMotion(
            controller,
            this::getAngle,
            this::setVoltage,
            (double t) -> {
                var setpoint = profile.calculate(t, start, goal);
                return new Pair<>(setpoint.position, setpoint.velocity);
            },
            this
        ).finallyDo(this::stop);
    }

}
