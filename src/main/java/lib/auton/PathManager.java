package lib.auton;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.Command;
import lib.commands.NoOp;
import org.json.simple.parser.ParseException;
import robot.Robot;
import robot.RobotMap.Dashboard;

import java.io.IOException;
import java.util.NoSuchElementException;

public final class PathManager {
    private PathManager() {}

    /**
     * When {@code false}, PathPlanner paths will be moved so that the starting position
     * of the path lines up with the robot position when the auton starts.
     * <p>
     * When {@code true}, paths will not be moved, and the robot will try to get to the
     * absolute position of the path on the field.
     */
    public static final boolean ABSOLUTE_PATHPLANNER_POSITIONING = true;

    public static final double PATHPLANNER_SLOWDOWN_FACTOR = 3;

    public static boolean shouldFlip() {
        return Robot.isRedAlliance();
    }

    // apparently cannot be higher than 85 (????) - see javadoc for FieldObject2d.setPoses()
    public static final int PATHPLANNER_PREVIEW_STEPS = 50;

    static final FieldObject2d liveTraj = Dashboard.liveField.getObject("auton_traj");
    static final FieldObject2d liveTarget = Dashboard.liveField.getObject("auton_next");

    static RobotConfig pathPlannerConfig;
    static {
        try {
            pathPlannerConfig = RobotConfig.fromGUISettings();
        } catch (IOException | ParseException e) {
            System.err.println("Failed to load PathPlanner config:\n" + e.getMessage());
        }
    }

    private static int pathCount = 0;

    public static Command c_path(String name) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(name);
            PathPlannerTrajectory traj = path.getIdealTrajectory(pathPlannerConfig).orElseThrow();
            System.out.printf("Loaded PathPlanner path '%s' (#%d).", name, ++pathCount);
            return new PathPlannerCommand(traj);
        } catch (IOException | ParseException e) {
            System.err.println("Failed to load PathPlanner path '" + name + "':\n" + e.getMessage());
        } catch (NoSuchElementException e) {
            System.err.println("Failed to load PathPlanner path '" + name + "'. Paths must have an ideal starting state.");
        }

        return new NoOp();
    }

    public static Pose2d[] makeTrajPreview(PathPlannerTrajectory traj, int steps) {
        return makeTrajPreview(traj, traj.getInitialPose(), steps);
    }

    public static Pose2d[] makeTrajPreview(PathPlannerTrajectory traj, Pose2d start, int steps) {
        double dur = traj.getTotalTimeSeconds();
        double timePerStep = dur / steps;

        Pose2d[] poses = new Pose2d[steps];
        for (int step = 0; step < steps; step++) {
            poses[step] = sampleTraj(traj, start, step * timePerStep);
        }
        return poses;
    }

    public static Pose2d sampleTraj(PathPlannerTrajectory traj, Pose2d start, double time) {
        Pose2d pose = traj.sample(time).pose, initial = traj.getInitialPose();
        if (start == initial) return pose; // should be exactly equal if getInitialPose() was used
        return start.plus(new Transform2d(initial, pose));
    }

}
