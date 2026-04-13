package lib.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Requires subsystems so no other code can interact with them.
 */
public class Idle extends Command {

    public Idle(Subsystem... requirements) {
        addRequirements(requirements);
    }
}
