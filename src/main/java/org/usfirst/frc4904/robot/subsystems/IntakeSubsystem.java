package org.usfirst.frc4904.robot.subsystems;

import org.usfirst.frc4904.standard.custom.motorcontrollers.SmartMotorController;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeSubsystem extends MotorSubsystem{
    public IntakeSubsystem(SmartMotorController intakeVerticalMotor, SmartMotorController intakeRollerMotor, CustomEncoder intakeEncoder){
        super(
            new SmartMotorController[] {intakeVerticalMotor, intakeRollerMotor},
            4
        );
    }

    public Command c_verticalDown () {
        return c_forward(true);
    }

    public Command c_verticalUp () {
        return c_backward(true);
    }
}
