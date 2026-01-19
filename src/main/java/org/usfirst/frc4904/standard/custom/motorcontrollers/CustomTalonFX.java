package org.usfirst.frc4904.standard.custom.motorcontrollers;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;

public class CustomTalonFX extends TalonFX implements SmartMotorController {

    public CustomTalonFX(int deviceId, boolean inverted) {
        super(deviceId);
        setInverted(inverted);
    }

    public CustomTalonFX(int deviceId) {
        this(deviceId, false);
    }

    /**
     * Setting to enable brake mode on neutral (when .neutralOutput(),
     * .disable(), or .stopMotor() is called, or when output percent is within
     * neutralDeadbandPercent of zero).
     *
     * This does not brake the motor. Use .neutralOutput() instead, after
     * setBrakeOnNeutral.
     */
    public void setBrakeOnNeutral() {
        setNeutralMode(NeutralModeValue.Brake);
    }

    /**
     * Setting to enable coast mode on neutral (when .neutralOutput(),
     * .disable(), or .stopMotor() is called, or when output percent is within
     * neutralDeadbandPercent of zero).
     *
     * This does not coast the motor. Use .neutralOutput() instead, after
     * setCoastOnNeutral.
     */
    public void setCoastOnNeutral() {
        setNeutralMode(NeutralModeValue.Coast);
    }

    // TODO: also support normally closed limit switches
    public boolean isFwdLimitSwitchPressed() {
        return getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;
    }
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

    private class ConfigSlot implements SmartMotorConfigSlot {

        private final SlotConfigs config = new SlotConfigs();

        ConfigSlot(int slot) {
            config.SlotNumber = slot;
        }

        @Override
        public SmartMotorConfigSlot setPID(double p, double i, double d) {
            config.kP = p;
            config.kI = i;
            config.kD = d;
            getConfigurator().apply(config);
            return this;
        }

        @Override
        public SmartMotorConfigSlot setElevFF(double kS, double kG, double kV, double kA) {
            config.kS = kS;
            config.kG = kG;
            config.kV = kV;
            config.kA = kA;
            config.GravityType = GravityTypeValue.Elevator_Static;
            getConfigurator().apply(config);
            return this;
        }

        @Override
        public SmartMotorConfigSlot setArmFF(double kS, double kG, double kV, double kA, double kCosRatio) {
            if (kCosRatio != 0) {
                throw new IllegalArgumentException("CustomTalonFX.configSlot(N).setArmFF() does not support kCosRatio. Call this method without the kCosRatio parameter disable this error, and use a different method of setting the ratio. Good luck");
            }
            config.kS = kS;
            config.kG = kG;
            config.kV = kV;
            config.kA = kA;
            config.GravityType = GravityTypeValue.Arm_Cosine;
            getConfigurator().apply(config);
            return this;
        }

        @Override
        public void holdPosition(double pos, double addedVoltage) {
            PositionVoltage request = new PositionVoltage(pos);
            request.Slot = config.SlotNumber;
            request.FeedForward = addedVoltage;
            setControl(request);
        }

        @Override
        public void holdVelocity(double vel, double addedVoltage) {
            VelocityVoltage request = new VelocityVoltage(vel);
            request.Slot = config.SlotNumber;
            request.FeedForward = addedVoltage;
            setControl(request);
        }
    }

    private static final int MAX_SLOTS = 3;
    private final ConfigSlot[] slots = new ConfigSlot[MAX_SLOTS];

    @Override
    public SmartMotorConfigSlot configSlot(int slot) {
        if (slot < 0 || slot >= MAX_SLOTS) {
            throw new IllegalArgumentException("CustomTalonFX slot must be between 0 and " + (MAX_SLOTS - 1) + " (inclusive).");
        }

        if (slots[slot] != null) {
            return slots[slot];
        } else {
            return slots[slot] = new ConfigSlot(slot);
        }
    }
}
