package org.usfirst.frc4904.robot.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.usfirst.frc4904.standard.commands.AsyncSequence;

import java.util.Arrays;

public interface TrajectoryCommand {

    double getDuration();

    // length of returned array may be slightly off due to rounding errors
    Pose2d[] getTrajPreview(int steps);

    default Pose2d[] getTrajPreview() {
        return getTrajPreview(PathManager.PATHPLANNER_PREVIEW_STEPS);
    }

    Pose2d getInitialPose();

    Pose2d getEndPose();

    /// COMMAND GROUP WRAPPERS

    /**
     * Note: Using {@link SequentialCommandGroup#addCommands(Command...) addCommands()} will not
     * add trajectories to the preview.
     */
    class SequentialPathPlannerGroup extends SequentialCommandGroup implements TrajectoryWrapper {

        private final double duration;
        private final TrajectoryCommand[] trajCommands;

        public SequentialPathPlannerGroup(Command... commands) {
            super(commands);

            trajCommands = filterTrajCommands(commands);

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

    private static TrajectoryCommand[] filterTrajCommands(Command... commands) {
        return Arrays.stream(commands)
                     .filter(cmd -> cmd instanceof TrajectoryCommand)
                     .toArray(TrajectoryCommand[]::new);
    }

}

// utility for wrapping existing command groups with TrajectoryCommand
interface TrajectoryWrapper extends TrajectoryCommand {

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
