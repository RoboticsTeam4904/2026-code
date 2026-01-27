package org.usfirst.frc4904.standard.humaninput;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.usfirst.frc4904.robot.RobotMap.Component;

/**
 * Operator specific version of HumanInterface
 */
public abstract class Operator extends HumanInput {

    public Operator(String name) {
        super(name);
    }

    protected static Command c_resetOdometry() {
        return new InstantCommand(() -> Component.chassis.resetOdometry());
    }
}
