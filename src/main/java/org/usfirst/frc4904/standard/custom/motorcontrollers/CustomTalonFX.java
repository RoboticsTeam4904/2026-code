package org.usfirst.frc4904.standard.custom.motorcontrollers;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import com.ctre.phoenix6.hardware.TalonFX;

public class CustomTalonFX extends TalonFX implements SmartMotorController {

    public CustomTalonFX(int deviceId, boolean inverted) {
        super(deviceId);

        setInverted(inverted);
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
    public boolean isFwdLimitSwitchPressed() throws IllegalAccessException {
        return getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;
    }
    public boolean isRevLimitSwitchPressed() throws IllegalAccessException {
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

    @Override
    public void neutralOutput() {
        stopMotor();
    }
}