package org.usfirst.frc4904.robot.humaninterface.operators;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.robot.RobotMap.Component;
import org.usfirst.frc4904.robot.subsystems.OrchestraSubsystem;
import org.usfirst.frc4904.standard.custom.motorcontrollers.CustomTalonFX;
import org.usfirst.frc4904.standard.humaninput.Operator;
import org.usfirst.frc4904.standard.util.Notifications;

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

        /// TEMPORARY INTAKE
        // joystick.button10.whileTrue(
        //     new ParallelCommandGroup(
        //         Component.intake.c_extend(),
        //         Component.intake.c_intake()
        //     )
        // );
        // joystick.button10.whileFalse(Component.intake.c_retract());

        /// NOTIFS TEST
        joystick.button10.onTrue(Notifications.c_testNotif());
        joystick.button11.onTrue(Notifications.c_sendRandom());

        /// ORCHESTRA
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

    }
}
