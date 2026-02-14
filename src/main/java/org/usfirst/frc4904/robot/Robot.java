/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/
package org.usfirst.frc4904.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.usfirst.frc4904.robot.RobotMap.Component;
import org.usfirst.frc4904.robot.humaninterface.drivers.SwerveGain;
import org.usfirst.frc4904.robot.humaninterface.operators.DefaultOperator;
import org.usfirst.frc4904.standard.CommandRobotBase;
import org.usfirst.frc4904.standard.commands.NoOp;

public class Robot extends CommandRobotBase {

    @Override
    public void initialize() {
        // BEGONE
        DriverStation.silenceJoystickConnectionWarning(true);

        SmartDashboard.putData("scheduler", CommandScheduler.getInstance());

        autonChooser.setDefaultOption("none", new NoOp());
        autonChooser.addOption("straight", Auton.c_jankStraight());
        autonChooser.addOption("reverse", Auton.c_jankReverse());
        Auton.initPathplanner(autonChooser, "STRET", "4", "7", "romtater", "aaahhh","4904");

        driverChooser.setDefaultOption("swerve", new SwerveGain());

        operatorChooser.setDefaultOption("default", new DefaultOperator());

        Component.chassis.startPoseEstimator(Pose2d.kZero);

    }

    @Override
    public void teleopInitialize() {
        // Component.lights.flashColor(LightSubsystem.Color.ENABLED);
    }

    @Override
    public void teleopExecute() {
        // Logging.log("place", Component.chassis.getPoseEstimate());
    }

    @Override
    public void teleopCleanup() { }

    @Override
    public void autonomousInitialize() { }

    @Override
    public void autonomousExecute() { }

    @Override
    public void autonomousCleanup() { }

    @Override
    public void disabledInitialize() {
        Component.vision.stopPositioning("Robot disabled", false);

        Component.chassis.stop();

        // Component.lights.flashColor(LightSubsystem.Color.DISABLED);
    }

    @Override
    public void disabledExecute() { }

    @Override
    public void disabledCleanup() {
        Component.chassis.setMotorBrake(true);
    }

    @Override
    public void testInitialize() { }

    @Override
    public void testExecute() { }

    @Override
    public void testCleanup() { }

    @Override
    public void alwaysExecute() {
        SmartDashboard.putNumber("navxTemp", Component.navx.getGyroTemperature());
        SmartDashboard.putNumber("match time", Timer.getMatchTime());

        // if (Logging.cooldown("Robot.alwaysExecute", 1)) {
        //     List<Tag> tags = Component.vision.gtm.getTags();
        //     if (!tags.isEmpty()) System.out.println("WE FOUND A TAG: " + tags);
        // }

        // if (Logging.cooldown("equis para nav", 0.5)) {
        //     int[] test = new int[8];
        //     int status = Component.navx.selfTest(test);
        //     System.out.println("navx test: " + status + " test: " + Arrays.toString(test));
        // }
        // Logging.log("navx rot", Component.navx.getYaw());
    }
}
