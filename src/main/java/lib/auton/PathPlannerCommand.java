package lib.auton;

import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import lib.commands.trajectory.TrajectoryCommand;
import lib.util.Util;
import robot.RobotMap;

class PathPlannerCommand extends Command implements TrajectoryCommand {

    public PathPlannerTrajectory traj;
    private final double duration;

    @Override
    public double getDuration() {
        return duration;
    }

    @Override
    public Pose2d[] getTrajPreview(int steps) {
        updateFlip();
        return PathManager.makeTrajPreview(traj, steps);
    }

    @Override
    public Pose2d getInitialPose() {
        updateFlip();
        return traj.getInitialPose();
    }

    @Override
    public Pose2d getEndPose() {
        updateFlip();
        return traj.getEndState().pose;
    }

    private double startTime;
    private Pose2d start;
    private boolean atEnd;

    private final Command gotoPoseCommand;

    PathPlannerCommand(PathPlannerTrajectory traj) {
        this.traj = traj;
        updateFlip();

        duration = traj.getTotalTimeSeconds();

        // effectively a WrapperCommand of this c_gotoPose() command
        // ...BUT WrapperCommand requires that the command is provided in the super() call,
        // which can't reference class fields like `startTime` or `offset`.
        // so i'll wrap it myself i guess.
        gotoPoseCommand = RobotMap.Component.chassis.c_gotoPose(() -> {
            if (atEnd) return null;

            double time = (Timer.getFPGATimestamp() - startTime) / PathManager.PATHPLANNER_SLOWDOWN_FACTOR;
            if (time >= duration) atEnd = true;

            Pose2d target = PathManager.sampleTraj(traj, start, time);
            PathManager.liveTarget.setPose(target);
            return target;
        }, true);
    }

    private boolean lastFlip;

    private void updateFlip() {
        if (lastFlip != PathManager.shouldFlip()) {
            lastFlip = !lastFlip;
            traj = traj.flip();
        }
    }

    @Override
    public void initialize() {
        updateFlip();

        atEnd = false;
        startTime = Timer.getFPGATimestamp();
        Pose2d current = RobotMap.Component.chassis.getPoseEstimate();

        start = PathManager.ABSOLUTE_PATHPLANNER_POSITIONING
            ? traj.getInitialPose()
            // orientation/rotation of path is always field relative
            : new Pose2d(current.getTranslation(), Rotation2d.kZero);

        int steps = Math.min(PathManager.PATHPLANNER_PREVIEW_STEPS, (int) Math.round(duration * 10));
        PathManager.liveTraj.setPoses(PathManager.makeTrajPreview(traj, start, steps));

        gotoPoseCommand.initialize();
    }

    @Override
    public void execute() {
        gotoPoseCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        Util.clearPose(PathManager.liveTraj, PathManager.liveTarget);

        gotoPoseCommand.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return gotoPoseCommand.isFinished();
    }

}
