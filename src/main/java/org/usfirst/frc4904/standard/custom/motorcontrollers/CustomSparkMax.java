package org.usfirst.frc4904.standard.custom.motorcontrollers;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;


public class CustomSparkMax extends SparkMax implements SmartMotorController {

    protected boolean limitSwitch;

    public CustomSparkMax(int deviceNumber, MotorType motorType, boolean inverted, boolean limitSwitch) {
        super(deviceNumber, motorType);

        setInverted(inverted);
        this.limitSwitch = limitSwitch;
    }

    public CustomSparkMax(int deviceNumber, MotorType motorType, boolean inverted) {
        this(deviceNumber, motorType, inverted, false);
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

    @SuppressWarnings("deprecation")
    public void setInverted(boolean inverted) {
        configure(new SparkMaxConfig().inverted(inverted));
    }

    @Override
    public void neutralOutput() {
        stopMotor();
    }

    @Override
    public boolean isFwdLimitSwitchPressed() throws IllegalAccessException {
        if (!limitSwitch) throw new IllegalAccessException("Cannot read limit switch state when CustomCANSparkMax was constructed without limit switch type!");
        return getForwardLimitSwitch().isPressed();
    }

    @Override
    public boolean isRevLimitSwitchPressed() throws IllegalAccessException {
        if (!limitSwitch) throw new IllegalAccessException("Cannot read limit switch state when CustomCANSparkMax was constructed without limit switch type!");
        return getReverseLimitSwitch().isPressed();
    }
}
