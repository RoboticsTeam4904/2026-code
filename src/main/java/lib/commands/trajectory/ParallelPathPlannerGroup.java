package lib.commands.trajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

/**
 * @see ParallelCommandGroup
 * @see TrajectoryCommand
 */
public class ParallelPathPlannerGroup extends ParallelCommandGroup implements TrajectoryCommand.Single {

    private final TrajectoryCommand trajCommand;

    public ParallelPathPlannerGroup(Command... commands) {
        super(commands);

        trajCommand = TrajectoryCommand.findTrajCommand(commands);

        if (trajCommand == null) {
            throw new IllegalArgumentException("ParallelPathPlannerGroup must be constructed with exactly one TrajectoryCommand");
        }
    }

    @Override
    public TrajectoryCommand getCommand() {
        return trajCommand;
    }

}
