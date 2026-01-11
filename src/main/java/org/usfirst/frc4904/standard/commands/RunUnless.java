package org.usfirst.frc4904.standard.commands;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.BooleanSupplier;

public class RunUnless extends RunIfElse {

    /**
     * Run a command based on a conditional callback. For example, if you only want
     * to shoot if a shooter is safe (based on its isUnsafe() function),
     * use: {@code new RunUnless(new Shoot(), shooter::isUnsafe)}
     * <p>
     * Conditions are AND-ed together (command will run if ANY are false).
     *
     * @param command    The command to be run if the condition is NOT met
     * @param conditions A condition function
     */
    public RunUnless(Command command, BooleanSupplier... conditions) {
        super(null, command, conditions);
    }
}
