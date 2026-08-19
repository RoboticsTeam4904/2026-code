package lib.commands.trajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * Note: Using {@link SequentialCommandGroup#addCommands(Command...) addCommands()} will not add trajectories to the
 * preview.
 *
 * @see SequentialCommandGroup
 * @see TrajectoryCommand
 */
public class SequentialPathPlannerGroup extends SequentialCommandGroup implements TrajectoryCommand.Multi {

    private final double duration;
    private final TrajectoryCommand[] trajCommands;

    public SequentialPathPlannerGroup(Command... commands) {
        super(commands);

        trajCommands = TrajectoryCommand.filterTrajCommands(commands);

        if (trajCommands.length == 0) {
            throw new IllegalArgumentException("Cannot construct a SequentialPathPlannerGroup without any TrajectoryCommands");
        }

        double dur = 0;
        for (var cmd : trajCommands) {
            dur += cmd.getDuration();
        }
        duration = dur;
    }

    @Override
    public double getDuration() {
        return duration;
    }

    @Override
    public TrajectoryCommand[] getCommands() {
        return trajCommands;
    }

}
