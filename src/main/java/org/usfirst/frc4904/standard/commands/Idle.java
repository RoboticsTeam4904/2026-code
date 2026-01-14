package org.usfirst.frc4904.standard.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Requires subsystems so no other code can interact with them.
 */
public class Idle extends Command {

    private final Runnable onInit;

    public Idle(Subsystem... requirements) {
        this(() -> {}, requirements);
    }

    public Idle(Runnable onInit, Subsystem... requirements) {
        this.onInit = onInit;
        addRequirements(requirements);
    }

    @Override
    public void initialize() {
        onInit.run();
    }
}
