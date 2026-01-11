package org.usfirst.frc4904.robot;

import edu.wpi.first.wpilibj2.command.*;
import org.usfirst.frc4904.robot.RobotMap.Component;
import org.usfirst.frc4904.robot.vision.VisionSubsystem.TagGroup;

public class Auton {

    public static boolean getFlipSide() {
        return Robot.AutonConfig.FLIP_SIDE;
    }

    // public static Command c_jank() {
    //     return new ParallelDeadlineGroup(
    //         new WaitCommand(1),
    //         new InstantCommand(() -> RobotMap.Component.chassis.drive(
    //             ChassisSpeeds.fromRobotRelativeSpeeds(
    //                 1.0,
    //                 0.0,
    //                 0.0,
    //                 Rotation2d.kZero
    //             )
    //         ))
    //     );
    // }

    /**
     * Move straight out of the starting zone and do nothing.
     */
    public static Command c_straight() {
        return Component.chassis.getAutonomousCommand("straight", true, false);
    }

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

    public static Command c_jankLeftCoral() {
        return c_jankSideCoral(1);
    }

    public static Command c_jankRightCoral() {
        return c_jankSideCoral(-1);
    }
}
