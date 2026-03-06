package org.usfirst.frc4904.robot.subsystems.IO;

public interface ArmIO {
    public ArmState.InputState getInstance();
    public void setArmState(ArmState.OutputState output); 
}
// defining your methods for inputs and outputs