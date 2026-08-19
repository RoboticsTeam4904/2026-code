package lib.commands.trajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import lib.auton.PathManager;

import java.util.Arrays;

/**
 * A command that includes <em>trajectory</em> data, most likely a PathPlanner path.
 * Command compositions can be replaced with a custom wrapper that serves the same
 * functionality but also implements this interface:
 * <ul>
 *     <li> {@link SequentialPathPlannerGroup}
 *     <li> {@link ParallelPathPlannerGroup}
 *     <li> {@link ParallelDeadlinePathPlannerGroup}
 *     <li> {@link AsyncPathPlannerSequence}
 * </ul>
 */
public interface TrajectoryCommand {

    double getDuration();

    // length of returned array may be slightly off due to rounding errors
    Pose2d[] getTrajPreview(int steps);

    default Pose2d[] getTrajPreview() {
        return getTrajPreview(PathManager.PATHPLANNER_PREVIEW_STEPS);
    }

    Pose2d getInitialPose();

    Pose2d getEndPose();

    static TrajectoryCommand[] filterTrajCommands(Command... commands) {
        return Arrays.stream(commands)
                     .filter(cmd -> cmd instanceof TrajectoryCommand)
                     .toArray(TrajectoryCommand[]::new);
    }

    static TrajectoryCommand findTrajCommand(Command... commands) {
        var trajCommands = filterTrajCommands(commands);
        return trajCommands.length == 1 ? trajCommands[0] : null;
    }

    // utility for wrapping existing parallel command groups with TrajectoryCommand
    interface Single extends TrajectoryCommand {

        TrajectoryCommand getCommand();

        @Override
        default double getDuration() {
            return getCommand().getDuration();
        }

        @Override
        default Pose2d[] getTrajPreview(int totalSteps) {
            return getCommand().getTrajPreview(totalSteps);
        }

        @Override
        default Pose2d getInitialPose() {
            return getCommand().getInitialPose();
        }

        @Override
        default Pose2d getEndPose() {
            return getCommand().getEndPose();
        }

    }

    // utility for wrapping existing sequential command groups with TrajectoryCommand
    interface Multi extends TrajectoryCommand {

        TrajectoryCommand[] getCommands();

        @Override
        default Pose2d[] getTrajPreview(int totalSteps) {
            return Arrays.stream(getCommands())
                         .flatMap(cmd -> {
                             int steps = (int) Math.round(cmd.getDuration() / getDuration() * totalSteps);
                             return Arrays.stream(cmd.getTrajPreview(steps));
                         })
                         .toArray(Pose2d[]::new);
        }

        @Override
        default Pose2d getInitialPose() {
            return getCommands()[0].getInitialPose();
        }

        @Override
        default Pose2d getEndPose() {
            var commands = getCommands();
            return commands[commands.length - 1].getEndPose();
        }

    }

}
