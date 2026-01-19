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

    public void setBrakeOnNeutral() {
        configure(new SparkMaxConfig().idleMode(IdleMode.kBrake));
    }

    public void setCoastOnNeutral() {
        configure(new SparkMaxConfig().idleMode(IdleMode.kCoast));
    }

    @SuppressWarnings("deprecation") // overrides deprecated method
    public void setInverted(boolean inverted) {
        configure(new SparkMaxConfig().inverted(inverted));
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

    private class ConfigSlot implements SmartMotorConfigSlot {

        private final SparkMaxConfig config = new SparkMaxConfig();
        private final ClosedLoopSlot slot;

        ConfigSlot(int slot) {
            this.slot = ClosedLoopSlot.fromInt(slot);
        }

        @Override
        public SmartMotorConfigSlot setPID(double p, double i, double d) {
            config.closedLoop.pid(p, i, d, slot);
            configure(config);
            return this;
        }

        @Override
        public SmartMotorConfigSlot setElevFF(double kS, double kG, double kV, double kA) {
            config.closedLoop.feedForward.kCos(0, slot);
            config.closedLoop.feedForward.svag(kS, kV, kA, kG, slot);
            configure(config);
            return this;
        }

        @Override
        public SmartMotorConfigSlot setArmFF(double kS, double kG, double kV, double kA, double kCosRatio) {
            config.closedLoop.feedForward.kG(0, slot);
            config.closedLoop.feedForward.svacr(kS, kV, kA, kG, kCosRatio, slot);
            configure(config);
            return this;
        }

        @Override
        public void holdPosition(double pos, double addedVoltage) {
            getClosedLoopController().setSetpoint(pos, ControlType.kPosition, slot, addedVoltage);
        }

        @Override
        public void holdVelocity(double vel, double addedVoltage) {
            getClosedLoopController().setSetpoint(vel, ControlType.kVelocity, slot, addedVoltage);
        }
    }

    private static final int MAX_SLOTS = 4;
    private final ConfigSlot[] slots = new ConfigSlot[MAX_SLOTS];

    @Override
    public SmartMotorConfigSlot configSlot(int slot) {
        if (slot < 0 || slot >= MAX_SLOTS) {
            throw new IllegalArgumentException("CustomSparkMax slot must be between 0 and " + (MAX_SLOTS - 1) + " (inclusive).");
        }

        if (slots[slot] != null) {
            return slots[slot];
        } else {
            return slots[slot] = new ConfigSlot(slot);
        }
    }
}
