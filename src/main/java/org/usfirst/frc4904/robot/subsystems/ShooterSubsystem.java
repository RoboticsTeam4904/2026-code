package org.usfirst.frc4904.robot.subsystems;

import org.usfirst.frc4904.standard.custom.motorcontrollers.SmartMotorController;

import edu.wpi.first.wpilibj2.command.Command;

public class ShooterSubsystem extends MotorSubsystem {
    public ShooterSubsystem(SmartMotorController shooterMotor1, SmartMotorController shooterMotor2) {
        super(
            new SmartMotorController[] {shooterMotor1, shooterMotor2}, 4
        );
    }

    public Command c_basicShoot(){
        return c_forward(true);
    }

    

}
