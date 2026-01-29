package org.usfirst.frc4904.robot.subsystems;

import org.usfirst.frc4904.standard.custom.motorcontrollers.SmartMotorController;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimberSubsystem extends MotorSubsystem{
    public ClimberSubsystem(SmartMotorController climbMotor, DutyCycleEncoder climbEncoder){
        super(
            climbMotor, 4
        );
    }

    public Command c_up(){
        return c_forward(true);
    }

    public Command c_down(){
        return c_backward(true);
    }

    

}
