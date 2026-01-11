package org.usfirst.frc4904.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.usfirst.frc4904.standard.commands.NoOp;

public final class CmdUtils {
    private CmdUtils() {
        throw new UnsupportedOperationException("Cannot instantiate utility class.");
    }

    public static void schedule(Command... commands) {
        CommandScheduler.getInstance().schedule(commands);
    }

    public static void cancelConflicting(Subsystem... requirements) {
        for (var subsystem : requirements) {
            var cmd = CommandScheduler.getInstance().requiring(subsystem);
            if (cmd != null) cmd.cancel();
        }
    }

    public static Command nonNull(Command cmd) {
        return cmd == null ? new NoOp() : cmd;
    }
}
