package org.usfirst.frc4904.standard.custom.motorcontrollers;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;


public class CustomSparkMax extends SparkMax implements SmartMotorController {

    protected final boolean limitSwitch;

    public CustomSparkMax(int deviceNumber, MotorType motorType, boolean inverted, boolean limitSwitch) {
        super(deviceNumber, motorType);

        setInverted(inverted);
        this.limitSwitch = limitSwitch;
    }

    public CustomSparkMax(int deviceNumber, MotorType motorType, boolean inverted) {
        this(deviceNumber, motorType, inverted, false);
    }

    public CustomSparkMax(int deviceNumber, MotorType motorType) {
        this(deviceNumber, motorType, false, false);
    }

    private void configure(SparkBaseConfig config) {
        configure(
            config,
            com.revrobotics.ResetMode.kNoResetSafeParameters,
            com.revrobotics.PersistMode.kPersistParameters
        );
    }

    public CustomSparkMax setBrakeOnNeutral() {
        configure(new SparkMaxConfig().idleMode(IdleMode.kBrake));
        return this;
    }

    public CustomSparkMax setCoastOnNeutral() {
        configure(new SparkMaxConfig().idleMode(IdleMode.kCoast));
        return this;
    }

    private boolean inverted = false;

    @SuppressWarnings("deprecation") // overrides deprecated method
    public void setInverted(boolean inverted) {
        this.inverted = inverted;
        configure(new SparkMaxConfig().inverted(inverted));
    }

    @SuppressWarnings("deprecation") // overrides deprecated method
    public boolean getInverted() {
        return inverted;
    }

    @Override
    public boolean isFwdLimitSwitchPressed() throws IllegalAccessException {
        if (!limitSwitch) throw new IllegalAccessException("Cannot read forward limit switch state when CustomSparkMax was constructed without limit switch.");
        return getForwardLimitSwitch().isPressed();
    }

    @Override
    public boolean isRevLimitSwitchPressed() throws IllegalAccessException {
        if (!limitSwitch) throw new IllegalAccessException("Cannot read reverse limit switch state when CustomSparkMax was constructed without limit switch.");
        return getReverseLimitSwitch().isPressed();
    }

    private final SparkMaxConfig config = new SparkMaxConfig();

    private double motorMechanismRatio = 1;
    private double continuousRange = 0;

    @Override
    public double getRotation() {
        // division is (in theory) handled by config.encoder.positionConversionFactor() below
        return getAbsoluteEncoder().getPosition(); //  / motorMechanismRatio;
    }

    @Override
    public CustomSparkMax setMotorMechanismRatio(double ratio) {
        motorMechanismRatio = ratio;
        double reciprocal = 1 / ratio;
        config.closedLoop.feedForward.kCosRatio(reciprocal);  // TODO reciprocal maybe wrong here? - also maybe not necessary at all if line below affects this (though i doubt it)
        config.encoder.positionConversionFactor(reciprocal);
        config.encoder.velocityConversionFactor(reciprocal);
        configure(config);
        return this;
    }

    @Override
    public double getMotorMechanismRatio() {
        return motorMechanismRatio;
    }

    @Override
    public CustomSparkMax setMechanismRotationOffset(double offset) {
        throw new UnsupportedOperationException("CustomSparkMax does not support setMechanismRotationOffset().");
    }

    @Override
    public double getMechanismRotationOffset() {
        System.err.println("CustomSparkMax does not support getMechanismRotationOffset().");
        return 0;
    }

    @Override
    public CustomSparkMax setPID(double p, double i, double d) {
        config.closedLoop.pid(p, i, d);
        configure(config);
        return this;
    }

    @Override
    public CustomSparkMax setElevFF(double kS, double kG, double kV, double kA) {
        config.closedLoop.feedForward.kCos(0);
        config.closedLoop.feedForward.svag(kS, kV, kA, kG);
        configure(config);
        return this;
    }

    @Override
    public CustomSparkMax setArmFF(double kS, double kG, double kV, double kA) {
        config.closedLoop.feedForward.kG(0);
        config.closedLoop.feedForward.kCos(kG);
        config.closedLoop.feedForward.sva(kS, kV, kA);
        configure(config);
        return this;
    }

    @Override
    public CustomSparkMax setContinuousRange(double range) {
        if (range < 0) {
            throw new IllegalArgumentException("CustomSparkMax.setContinuousRange() cannot be negative.");
        }

        continuousRange = range;
        if (range != 0) {
            config.closedLoop.positionWrappingEnabled(true);
            config.closedLoop.positionWrappingInputRange(0, range);
        } else {
            config.closedLoop.positionWrappingEnabled(false);
        }
        return this;
    }

    @Override
    public double getContinuousRange() {
        return continuousRange;
    }

    @Override
    public void holdPosition(double pos, double addedVoltage) {
        getClosedLoopController().setSetpoint(pos, ControlType.kPosition, ClosedLoopSlot.kSlot0, addedVoltage);
    }

    @Override
    public void holdVelocity(double vel, double addedVoltage) {
        getClosedLoopController().setSetpoint(vel, ControlType.kVelocity, ClosedLoopSlot.kSlot0, addedVoltage);
    }
}
