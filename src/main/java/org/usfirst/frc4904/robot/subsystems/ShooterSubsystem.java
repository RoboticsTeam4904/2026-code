package org.usfirst.frc4904.robot.subsystems;

import org.usfirst.frc4904.standard.custom.motorcontrollers.SmartMotorController;
import org.usfirst.frc4904.standard.custom.sensors.CustomEncoder;

public class ShooterSubsystem extends MotorSubsystem {
    public ShooterSubsystem(SmartMotorController shooterMotor, CustomEncoder shooterEncoder) {
        super(
            shooterMotor, 4
        );
    }
}
