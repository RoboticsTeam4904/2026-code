package lib.commands.conditional;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import lib.commands.NoOp;
import lib.util.Util;

import java.util.function.BooleanSupplier;

public class RunIf extends ConditionalCommand {

    /**
     * Run a command based on a conditional callback. For example, if you only want
     * to shoot if a shooter is ready (based on its isReady() function),
     * use: {@code new RunIf(new Shoot(), shooter::isReady)}
     * <p>
     * Conditions are AND-ed together (command will only run if ALL are true).
     *
     * @param command    The command to be run if the condition is met
     * @param conditions A variable number of condition functions
     */
    public RunIf(Command command, BooleanSupplier... conditions) {
        super(
            command,
            new NoOp(),
            Util.all(conditions)
        );
    }
}
