package org.usfirst.frc4904.standard.custom.motorcontrollers;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.MathUtil;

public class CustomTalonFX extends TalonFX implements SmartMotorController {

    public CustomTalonFX(int deviceId, boolean inverted) {
        super(deviceId);
        setInverted(inverted);
    }

    public CustomTalonFX(int deviceId) {
        this(deviceId, false);
    }

    /**
     * Setting to toggle brake mode on neutral.
     * <p>
     * This does not brake the motor. Use {@code .neutralOutput()} instead, after {@code .setMotorBrake(true)}.
     */
    @Override
    public void setMotorBrake(boolean brake) {
        setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    // TODO: also support normally closed limit switches
    @Override
    public boolean isFwdLimitSwitchPressed() {
        return getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;
    }
    @Override
    public boolean isRevLimitSwitchPressed() {
        // TODO: this boolean might be reversed
        return getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
    }

    private boolean inverted = false;

    /**
     * @param inverted Whether to invert - true is clockwise positive, false is counterclockwise positive (default).
     */
    @Override
    public void setInverted(boolean inverted) {
        this.inverted = inverted;
        // CounterClockwise_Positive seems to be the default value
        InvertedValue direction = inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        getConfigurator().apply(new MotorOutputConfigs().withInverted(direction));
    }

    @Override
    public boolean getInverted() {
        return inverted;
    }

    private final Slot0Configs pidfConfig = new Slot0Configs();

    private double motorMechanismRatio = 1;
    private double mechanismRotationOffset = 0;
    private double continuousRange = 0;

    @Override
    public double getRotation() {
        double rot = getPosition().getValueAsDouble();
        return rot / motorMechanismRatio + mechanismRotationOffset;
    }

    private void updateMotorMechanismRatio() {
        double ratio = motorMechanismRatio * (continuousRange != 0 ? continuousRange : 1);
        getConfigurator().apply(new FeedbackConfigs().withSensorToMechanismRatio(ratio));
    }

    private void continuousRangeArmFFError() {
        // suboptimal edge case due to questionable implementation of continuous range
        // i can't imagine any use case for this combination so it's probably fine
        System.err.println("CustomTalonFX with arm FeedForward is not compatible with continuous range ≠ 1.\n"
                         + "FeedForward kG calculations will be incorrectly scaled by continuous range.");
    }

    @Override
    public CustomTalonFX setMotorMechanismRatio(double ratio) {
        motorMechanismRatio = ratio;
        updateMotorMechanismRatio();
        return this;
    }

    @Override
    public double getMotorMechanismRatio() {
        return motorMechanismRatio;
    }

    private boolean gravityNegated = false;

    @Override
    public CustomTalonFX setMechanismRotationOffset(double offset) {
        mechanismRotationOffset = offset;

        // GravityArmPositionOffset only accepts values within ±0.25 for some unknown reason
        // for values 0.25-0.75, subtract 0.5 and flip kG which is (hopefully) equivalent
        offset = MathUtil.inputModulus(offset, -0.25, 0.75);
        boolean flipGravity = offset > 0.25;
        if (flipGravity != gravityNegated) {
            pidfConfig.kG *= -1;
            gravityNegated = flipGravity;
        }
        if (flipGravity) offset -= 0.5;

        pidfConfig.GravityArmPositionOffset = offset;
        getConfigurator().apply(pidfConfig);
        return this;
    }

    @Override
    public double getMechanismRotationOffset() {
        return mechanismRotationOffset;
    }

    @Override
    public CustomTalonFX setPID(double p, double i, double d) {
        pidfConfig.kP = p;
        pidfConfig.kI = i;
        pidfConfig.kD = d;
        getConfigurator().apply(pidfConfig);
        return this;
    }

    @Override
    public CustomTalonFX setElevFF(double kS, double kG, double kV, double kA) {
        pidfConfig.kS = kS;
        pidfConfig.kG = kG;
        pidfConfig.kV = kV;
        pidfConfig.kA = kA;
        pidfConfig.GravityType = GravityTypeValue.Elevator_Static;
        getConfigurator().apply(pidfConfig);
        return this;
    }

    @Override
    public CustomTalonFX setArmFF(double kS, double kG, double kV, double kA) {
        if (continuousRange != 0 && continuousRange != 1) {
            continuousRangeArmFFError();
        }
        pidfConfig.kS = kS;
        pidfConfig.kG = gravityNegated ? -kG : kG;
        pidfConfig.kV = kV;
        pidfConfig.kA = kA;
        pidfConfig.GravityType = GravityTypeValue.Arm_Cosine;
        getConfigurator().apply(pidfConfig);
        return this;
    }

    @Override
    public CustomTalonFX setContinuousRange(double range) {
        if (range < 0) {
            throw new IllegalArgumentException("CustomTalonFX.setContinuousRange() cannot be negative.");
        }
        if (range != 0 && range != 1 && pidfConfig.GravityType == GravityTypeValue.Arm_Cosine) {
            continuousRangeArmFFError();
        }

        continuousRange = range;
        updateMotorMechanismRatio();
        getConfigurator().apply(new ClosedLoopGeneralConfigs().withContinuousWrap(range != 0));
        return this;
    }

    @Override
    public double getContinuousRange() {
        return continuousRange;
    }

    private final PositionVoltage posRequest = new PositionVoltage(0);
    private final VelocityVoltage velRequest = new VelocityVoltage(0);

    @Override
    public void holdPosition(double pos, double addedVoltage) {
        if (continuousRange != 0) {
            double current = getRotation();
            double dist = continuousRange / 2;
            pos = MathUtil.inputModulus(pos, current - dist, current + dist);
        }
        double rawPos = (pos - mechanismRotationOffset) * motorMechanismRatio;
        setControl(posRequest.withPosition(rawPos).withFeedForward(addedVoltage));
    }

    @Override
    public void holdVelocity(double vel, double addedVoltage) {
        setControl(velRequest.withVelocity(vel * motorMechanismRatio).withFeedForward(addedVoltage));
    }
}
