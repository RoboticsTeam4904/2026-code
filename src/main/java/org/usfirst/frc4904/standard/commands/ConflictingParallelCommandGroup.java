package org.usfirst.frc4904.standard.commands;

import java.util.Arrays;
import java.util.Collections;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WrapperCommand;

public class ConflictingParallelCommandGroup extends ParallelCommandGroup {

    public ConflictingParallelCommandGroup(Command... commands) {
        super(
            Arrays.stream(commands)
                  .map(cmd -> new WrapperCommand(cmd) {
                      @Override
                      public Set<Subsystem> getRequirements() {
                          return Collections.emptySet();
                      }
                  })
                  .toArray(Command[]::new)
        );

        for (var cmd : commands) {
            addRequirements(cmd.getRequirements());
        }
    }

}
