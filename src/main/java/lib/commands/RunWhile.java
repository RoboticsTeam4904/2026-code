package lib.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;

import java.util.function.BooleanSupplier;

public class RunWhile extends ParallelRaceGroup {

    public RunWhile(Command command, BooleanSupplier condition) {
        super(command, new WaitWhileCommand(condition));
    }
}
