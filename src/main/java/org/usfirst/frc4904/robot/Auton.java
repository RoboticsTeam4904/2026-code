package org.usfirst.frc4904.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import org.json.simple.parser.ParseException;
import org.usfirst.frc4904.robot.RobotMap.Component;
import org.usfirst.frc4904.robot.swerve.SwerveSubsystem;
import org.usfirst.frc4904.standard.commands.NoOp;
import org.usfirst.frc4904.standard.util.Util;

import java.io.IOException;
import java.util.NoSuchElementException;

public final class Auton {

    private static final double PATHPLANNER_SLOWDOWN_FACTOR = 1.5;

    private Auton() {}

    private static final Field2d field = new Field2d();
    private static final FieldObject2d fieldTraj = field.getObject("traj");

    static {
        SmartDashboard.putData("auton/field", field);
    }

    /**
     * Move straight out of the starting zone and do nothing.
     */
    // public static Command c_straight() {
    //     return Component.chassis.getAutonomousCommand("straight", true, false);
    // }

   // actually moves backwards - robot must be placed physically backwards on the field
    public static Command c_jankStraight() {
        return new SequentialCommandGroup(
            new WaitCommand(12),
            Component.chassis.c_driveRobotRelative(-0.5, 0, 0).withTimeout(2),
            Component.chassis.c_stop()
        );
    }

    // actually moves forwards
    public static Command c_jankReverse() {
        return new SequentialCommandGroup(
            new WaitCommand(12),
            Component.chassis.c_driveRobotRelative(0.5, 0, 0).withTimeout(2),
            Component.chassis.c_stop()
        );
    }

    /// PATHPLANNER

    static RobotConfig pathPlannerConfig;
    static {
        try {
            pathPlannerConfig = RobotConfig.fromGUISettings();
        } catch (IOException | ParseException e) {
            System.err.println("Failed to load PathPlanner config:\n" + e.getMessage());
        }
    }

    public static void initPathplanner(SendableChooser<? super Command> autonChooser, String... names) {
        SwerveSubsystem swerve = Component.chassis;

        if (pathPlannerConfig == null) return;

        AutoBuilder.configure(
            swerve::getPoseEstimate,
            swerve::resetPose,
            swerve::getChassisSpeeds,
            (speeds, feedforwards) -> swerve.driveRobotRelative(speeds),
            new PPHolonomicDriveController(
                new PIDConstants(5, 0, 0),
                new PIDConstants(5, 0, 0)
            ),
            pathPlannerConfig,
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
            },
            swerve
        );

        for (var name : names) {
            autonChooser.addOption(name, c_pathPlanner(name));
            // autonChooser.addOption(name, new PathPlannerAuto(name));
        }
    }

    public static Command c_pathPlanner(String file) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(file);
            PathPlannerTrajectory traj = path.getIdealTrajectory(pathPlannerConfig).orElseThrow();
            double dur = traj.getTotalTimeSeconds();

            final var holder = new Object() { double startTime; Translation2d offset; boolean atEnd; };

            return new ParallelCommandGroup(
                new InstantCommand(
                    // SETUP:
                    () -> {
                        holder.startTime = Timer.getFPGATimestamp();
                        Pose2d current = Component.chassis.getPoseEstimate(), initial = traj.getInitialPose();
                        holder.offset = current.getTranslation().minus(initial.getTranslation());
                        holder.atEnd = false;

                        int steps = Util.clamp((int) Math.round(dur * 8), 2, 50); // 8 steps per second, 50 max
                        double timePerStep = dur / steps;

                        Pose2d[] poses = new Pose2d[steps];
                        for (int step = 0; step < steps; step++) {
                            poses[step] = sampleTraj(traj, step * timePerStep, holder.offset);
                        }
                        fieldTraj.setPoses(poses);
                    }
                ),
                // RUN CONTINUOUSLY:
                Component.chassis.c_gotoPose(() -> {
                    double time = (Timer.getFPGATimestamp() - holder.startTime) / PATHPLANNER_SLOWDOWN_FACTOR;
                    if (time >= dur) {
                        if (holder.atEnd) {
                            return null;
                        } else {
                            time = dur;
                            holder.atEnd = true;
                        }
                    };

                    Pose2d target = sampleTraj(traj, time, holder.offset);
                    field.setRobotPose(target);
                    return target;
                })
            ).finallyDo(
                // CLEANUP:
                () -> {
                    fieldTraj.setPoses();
                    field.getRobotObject().setPoses();
                }
            );
        } catch (IOException | ParseException e) {
            System.err.println("Failed to load PathPlanner path '" + file + "':\n" + e.getMessage());
        } catch (NoSuchElementException e) {
            System.err.println("Failed to load PathPlanner path '" + file + "'. Paths must have an ideal starting state.");
        }

        return new NoOp();
    }

    private static Pose2d sampleTraj(PathPlannerTrajectory traj, double time, Translation2d offset) {
        Pose2d idealPose = traj.sample(time).pose;
        return new Pose2d(idealPose.getX() + offset.getX(), idealPose.getY() + offset.getY(), idealPose.getRotation());
    }
}
