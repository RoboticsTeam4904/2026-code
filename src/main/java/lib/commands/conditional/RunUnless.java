package lib.commands.conditional;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import lib.commands.NoOp;
import lib.util.Util;

import java.util.function.BooleanSupplier;

public class RunUnless extends ConditionalCommand {

    /**
     * Run a command based on a conditional callback. For example, if you only want
     * to shoot if a shooter is safe (based on its isUnsafe() function),
     * use: {@code new RunUnless(new Shoot(), shooter::isUnsafe)}
     * <p>
     * Conditions are OR-ed together (command will only run if ALL are false).
     *
     * @param command    The command to be run if the condition is NOT met
     * @param conditions A condition function
     */
    public RunUnless(Command command, BooleanSupplier... conditions) {
        super(
            new NoOp(),
            command,
            Util.any(conditions)
        );
    }
}
