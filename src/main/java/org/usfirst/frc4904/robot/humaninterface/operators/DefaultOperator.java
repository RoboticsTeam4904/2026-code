package org.usfirst.frc4904.robot.humaninterface.operators;

import java.util.List;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.robot.RobotMap.Component;
import org.usfirst.frc4904.robot.vision.GoogleTagManager.Tag;
import org.usfirst.frc4904.standard.humaninput.Operator;

import edu.wpi.first.math.util.Units;

public class DefaultOperator extends Operator {

    public DefaultOperator() {
        super("DefaultOperator");
    }

    public DefaultOperator(String name) {
        super(name);
    }

    @Override
    public void bindCommands() {
    }
}
