package lib.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;

import java.util.function.Supplier;

public class CreateOnInitialize extends DeferredCommand {

    /**
     * Wrapper for {@link DeferredCommand} that immediately gets a command from
     * the supplier and uses it to determine this command's requirements.
     * If you do not need this behavior, you can use {@link DeferredCommand}
     * directly instead and manually supply the necessary requirements.
     *
     * @param supplier The command supplier
     */
    public CreateOnInitialize(Supplier<Command> supplier) {
        super(supplier, supplier.get().getRequirements());
    }
}
