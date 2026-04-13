package lib.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import java.util.function.BooleanSupplier;

public class RunUntil extends ParallelRaceGroup {

    public RunUntil(Command command, BooleanSupplier condition) {
        super(command, new WaitUntilCommand(condition));
    }
}
