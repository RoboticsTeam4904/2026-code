package org.usfirst.frc4904.robot.subsystems;

import org.usfirst.frc4904.standard.custom.motioncontrollers.ezControl;
import org.usfirst.frc4904.standard.custom.motioncontrollers.ezMotion;
import org.usfirst.frc4904.standard.custom.motorcontrollers.SmartMotorController;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;

public class IntakeSubsystem extends MotorSubsystem {

    // TODO change all these
    public static final double kP = 15;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;
    public static final double kG = 0;

    public static final double retractAngle = 0.4;
    public static final double extendAngle = 0.7;

    public static final double MAX_VEL = 8;
    public static final double MAX_ACCEL = MAX_VEL * 4; // accelerate to max speed in 1/4 of a second

    private final SmartMotorController verticalMotor;
    private final DutyCycleEncoder encoder;
    private final ArmFeedforward feedforward;

    public IntakeSubsystem(
        SmartMotorController verticalMotor,
        SmartMotorController rollerMotor,
        DutyCycleEncoder encoder
    ) {
        super(rollerMotor, 4);

        this.verticalMotor = verticalMotor;
        this.encoder = encoder;
        this.feedforward = new ArmFeedforward(kS, kG, kV, kA);
    }

    public double getAngle() {
        // TODO maybe not negative
        return -encoder.get();
    }
    
    public Command c_intake() {
        return c_forward(true);
    }

    // TODO: actually find the angles
    public Command c_gotoAngle(double angle) {
        var pid = new PIDController(kP, kI, kD);
        pid.enableContinuousInput(0, 1);
        ezControl controller = new ezControl(
            pid,
            (position, velocity) -> feedforward.calculate(getAngle(), velocity)
        );
        var constraints = new TrapezoidProfile.Constraints(MAX_VEL, MAX_ACCEL);

        return new ezMotion(
            controller,
            this::getAngle,
            verticalMotor::setVoltage,
            angle,
            constraints,
            this
        ).alongWith(new PrintCommand("going to angle: " + angle))
         .finallyDo(() -> {stop(); System.out.println("went to angle: " + angle);});
    }

    public Command c_extend() {
        return c_gotoAngle(extendAngle);
    }

    public Command c_retract() {
        return c_gotoAngle(retractAngle);
    }
}

