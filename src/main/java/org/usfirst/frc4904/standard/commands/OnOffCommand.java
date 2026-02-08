package org.usfirst.frc4904.standard.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import org.usfirst.frc4904.standard.util.CmdUtil;

public class OnOffCommand extends Command {

    private final Command on;
    private final Command off;

    /**
     * @param on Command that will be initialized when this command initializes,
     *           and canceled when this command is canceled
     * @param off Opposite of {@code on} - runs whenever this command is not running
     */
    public OnOffCommand(Command on, Command off) {
        super();

        // wrap commands so that they are marked as composed and cannot be scheduled elsewhere
        // however the private wrapper commands can still be scheduled within this command
        this.on = new WrapperCommand(on) {};
        this.off = new WrapperCommand(off) {};
    }

    @Override
    public final void initialize() {
        off.cancel();
        CmdUtil.schedule(on);
    }

    @Override
    public final void end(boolean interrupted) {
        on.cancel();
        CmdUtil.schedule(off);
    }

    // does not need to inherit requirements or interruption behavior
    // since that will be handled individually by the currently running command (on or off)

    @Override
    public boolean runsWhenDisabled() {
        return true; // handled individually by on/off commands
    }

}
