package org.usfirst.frc4904.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import org.json.simple.parser.ParseException;
import org.usfirst.frc4904.robot.RobotMap.Component;
import org.usfirst.frc4904.robot.swerve.SwerveSubsystem;
import org.usfirst.frc4904.standard.commands.NoOp;

import javax.xml.crypto.dsig.Transform;
import java.io.IOException;
import java.util.List;
import java.util.NoSuchElementException;

public final class Auton {
    private Auton() {}

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

    public static void initPathplanner(SendableChooser<? super Command> autonChooser) {
        initPathplanner(autonChooser, AutoBuilder.getAllAutoNames());
    }

    public static void initPathplanner(SendableChooser<? super Command> autonChooser, List<String> names) {
        SwerveSubsystem swerve = Component.chassis;

        if (pathPlannerConfig == null) return;

        AutoBuilder.configure(
            swerve::getPoseEstimate,
            swerve::resetPose,
            swerve::getChassisSpeeds,
            (speeds, feedforwards) -> swerve.driveRobotRelative(speeds),
            new PPHolonomicDriveController(
                new PIDConstants(5.0, 0.0, 0.0),
                new PIDConstants(5.0, 0.0, 0.0)
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
            // autonChooser.addOption(name, c_pathPlanner(name));
            autonChooser.addOption(name, new PathPlannerAuto(name));
        }
    }

    public static Command c_pathPlanner(String file) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile("MyPath");
            PathPlannerTrajectory traj = path.getIdealTrajectory(pathPlannerConfig).orElseThrow();

            final var holder = new Object() { double startTime; Transform2d offset; };

            return new WrapperCommand(Component.chassis.c_gotoPose(() -> {
                double now = Timer.getFPGATimestamp();
                Pose2d idealPose = traj.sample(now - holder.startTime).pose;
                return idealPose.plus(holder.offset);
            })) {
                @Override
                public void initialize() {
                    holder.startTime = Timer.getFPGATimestamp();
                    holder.offset = Component.chassis.getPoseEstimate().minus(traj.getInitialPose());
                    super.initialize();
                }
            };
        } catch (IOException | ParseException e) {
            System.err.println("Failed to load PathPlanner path '" + file + "':\n" + e.getMessage());
        } catch (NoSuchElementException e) {
            System.err.println("Failed to load PathPlanner path '" + file + "'. Paths must have an ideal starting state.");
        }

        return new NoOp();
    }
}
