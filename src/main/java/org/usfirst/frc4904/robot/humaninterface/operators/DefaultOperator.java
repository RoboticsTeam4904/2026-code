package org.usfirst.frc4904.robot.humaninterface.operators;

import java.util.List;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.robot.RobotMap.Component;
import org.usfirst.frc4904.robot.subsystems.OrchestraSubsystem;
import org.usfirst.frc4904.robot.vision.GoogleTagManager.Tag;
import org.usfirst.frc4904.standard.custom.motorcontrollers.CustomTalonFX;
import org.usfirst.frc4904.standard.humaninput.Operator;
import org.usfirst.frc4904.standard.util.Elastic;

import com.ctre.phoenix6.Orchestra;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;

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

        CustomTalonFX[] motors = {
            Component.flDrive,
            Component.frDrive,
            Component.blDrive,
            Component.brDrive,
            Component.flTurn,
            null, null, null,
            Component.frTurn,
            null, null, null,
            Component.blTurn,
            null, null, null,
            Component.brTurn
        };

        joystick.button7.onTrue(OrchestraSubsystem.c_loadAndPlaySong(
            "shreksophone",
            4,
            motors
        ));
        joystick.button8.onTrue(OrchestraSubsystem.c_loadAndPlaySong(
            "coconutNyoom",
            4,
            motors
        ));

        joystick.button12.onTrue(new InstantCommand(OrchestraSubsystem::stopAll));

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

        // NOTIFS TEST
        joystick.button10.onTrue(Elastic.c_testNotif());
        joystick.button11.onTrue(Elastic.c_bensults());
       
    }
}
