package lib.commands.trajectory;

import edu.wpi.first.wpilibj2.command.Command;
import lib.commands.AsyncSequence;

/**
 * Note: Path trajectory is only assembled from synchronous commands.
 *
 * @see AsyncSequence
 * @see TrajectoryCommand
 */
public class AsyncPathPlannerSequence extends AsyncSequence implements TrajectoryCommand.Multi {

    private final double duration;
    private final TrajectoryCommand[] trajCommands;

    public AsyncPathPlannerSequence(Command... commands) {
        super(commands);

        trajCommands = TrajectoryCommand.filterTrajCommands(commands);

        if (trajCommands.length == 0) {
            throw new IllegalArgumentException("Cannot construct an AsyncPathPlannerSequence without any synchronous TrajectoryCommands");
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
