package lib.commands.trajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;

/**
 * @see ParallelDeadlineGroup
 * @see TrajectoryCommand
 */
public class ParallelDeadlinePathPlannerGroup extends ParallelDeadlineGroup implements TrajectoryCommand.Single {

    private final TrajectoryCommand trajCommand;

    public ParallelDeadlinePathPlannerGroup(Command deadline, Command... otherCommands) {
        super(deadline, otherCommands);

        Command[] commands = new Command[otherCommands.length + 1];
        commands[0] = deadline;
        System.arraycopy(otherCommands, 0, commands, 1, otherCommands.length);
        trajCommand = TrajectoryCommand.findTrajCommand(commands);

        if (trajCommand == null) {
            throw new IllegalArgumentException("ParallelDeadlinePathPlannerGroup must be constructed with exactly one TrajectoryCommand");
        }
    }

    @Override
    public TrajectoryCommand getCommand() {
        return trajCommand;
    }

}
