package org.usfirst.frc4904.standard.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * InstantCommand that can run when the robot is disabled.
 * Useful for things like zeroing motors that do NOT move any part of the robot.
 */
public class AlwaysRunnableInstantCommand extends InstantCommand {

    public AlwaysRunnableInstantCommand(Runnable toRun, Subsystem... requirements) {
        super(toRun, requirements);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

}
