package org.usfirst.frc4904.robot;

import edu.wpi.first.wpilibj2.command.Command;
import org.usfirst.frc4904.standard.commands.NoOp;

public class CmdUtils {

    public static Command nonNull(Command cmd) {
        return cmd == null ? new NoOp() : cmd;
    }
}
