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
        var joystick = RobotMap.HumanInput.Operator.joystick;
        var xyJoystick = RobotMap.HumanInput.Driver.xyJoystick;
        var turnJoystick = RobotMap.HumanInput.Driver.turnJoystick;

        /// VISION
        turnJoystick.button1.onTrue(c_flipZero());
        turnJoystick.button2.whileTrue(Component.chassis.c_rotateTo(() -> {
            List<Tag> tags = Component.vision.gtm.getTags();
            if (tags.isEmpty()) return null;
            Tag tag = tags.get(0);
            return Units.radiansToRotations(
                Math.atan2(tag.pos().getY() + 0.25, tag.pos().getX())
            );
        }));

        /// ODOMETRY RESETTING
        xyJoystick.button1.onTrue(c_resetOdometry());
        xyJoystick.button2.onTrue(c_zeroSwerve());
    }
}
