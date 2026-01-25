package org.usfirst.frc4904.robot.humaninterface.operators;

import org.usfirst.frc4904.robot.RobotMap.Component;
import org.usfirst.frc4904.robot.RobotMap.HumanInput;
import org.usfirst.frc4904.robot.vision.VisionSubsystem.TagGroup;
import org.usfirst.frc4904.standard.humaninput.Operator;

public class AnnaOperator extends Operator {

    public AnnaOperator() {
        super("AnnaOperator");
    }

    public AnnaOperator(String name) {
        super(name);
    }

    @Override
    public void bindCommands() {
        var joystick = HumanInput.Operator.joystick;
        var xyJoystick = HumanInput.Driver.xyJoystick;
        var turnJoystick = HumanInput.Driver.turnJoystick;

        /// VISION
        turnJoystick.button1.whileTrue(Component.vision.c_align(TagGroup.ANY, 0));

        /// ODOMETRY RESETTING
        xyJoystick.button1.onTrue(c_resetOdometry());
        // xyJoystick.button2.onTrue(c_zeroSwerve());
    }
}
