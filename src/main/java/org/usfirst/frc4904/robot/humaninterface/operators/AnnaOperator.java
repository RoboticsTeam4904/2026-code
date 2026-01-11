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

       
        /// MANUAL RAMP CONTROL
        joystick.button6.whileTrue(Component.ramp.c_forward(true));
        joystick.button4.whileTrue(Component.ramp.c_backward(true));

        /// MANUAL OUTTAKE CONTROL
        joystick.button3.whileTrue(Component.outtake.c_backward(true));
        joystick.button5.whileTrue(Component.outtake.c_forward(true));

        /// VISION
        turnJoystick.button1.whileTrue(Component.vision.c_align(TagGroup.ANY, -1));
        turnJoystick.button2.whileTrue(Component.vision.c_align(TagGroup.ANY, 1));

        /// ODOMETRY RESETTING
        xyJoystick.button1.onTrue(c_resetOdometry());

    }
}
