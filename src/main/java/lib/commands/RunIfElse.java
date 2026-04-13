package lib.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import lib.util.CmdUtil;
import lib.util.Util;

import java.util.function.BooleanSupplier;

public class RunIfElse extends ConditionalCommand {
    /**
     * Combination of {@link RunIf} and {@link RunUnless}.
     * <p>
     * Conditions are AND-ed together:
     * <ul>
     *   <li> onTrue will only run if ALL conditions are true
     *   <li> onFalse will run if ANY conditions are false
     * </ul>
     */
    public RunIfElse(Command onTrue, Command onFalse, BooleanSupplier... conditions) {
        super(
            CmdUtil.nonNull(onTrue),
            CmdUtil.nonNull(onFalse),
            Util.all(conditions)
        );
    }
}
