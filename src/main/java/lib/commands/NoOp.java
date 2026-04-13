package lib.commands;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Does nothing.
 */
public class NoOp extends Command {

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

}
