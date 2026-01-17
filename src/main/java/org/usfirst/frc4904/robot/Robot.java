/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/
package org.usfirst.frc4904.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.usfirst.frc4904.robot.RobotMap.Component;
import org.usfirst.frc4904.robot.humaninterface.drivers.SwerveGain;
import org.usfirst.frc4904.robot.humaninterface.operators.AnnaOperator;
import org.usfirst.frc4904.robot.humaninterface.operators.DefaultOperator;
import org.usfirst.frc4904.robot.vision.GoogleTagManager.Tag;
import org.usfirst.frc4904.standard.CommandRobotBase;
import org.usfirst.frc4904.standard.commands.NoOp;
import org.usfirst.frc4904.standard.util.CmdUtil;

import java.util.List;
import java.util.function.Supplier;

public class Robot extends CommandRobotBase {

    public static class AutonConfig {

        /** Whether to run auton at all */
        public static final boolean ENABLED = true;

        /** Whether to flip the path to the other side of the current alliance's field */
        public static final boolean FLIP_SIDE = false;

        /** The auton to run */
        public static Supplier<Command> COMMAND = Auton::c_jankStraight;
    }

    private final SwerveGain driver = new SwerveGain();
    private final DefaultOperator operator = new DefaultOperator();
    private final RobotMap map = new RobotMap(); // TODO move init code to static method and put in initialize()

    @Override
    public void initialize() {
        autoChooser.setDefaultOption("none", new NoOp());
        autoChooser.addOption("straight", Auton.c_straight());
        autoChooser.addOption("reverse", Auton.c_jankReverse());
    }

    @Override
    public void teleopInitialize() {
        // TODO move to initialize() and check that that doesnt cause any problems
        driver.bindCommands();
        operator.bindCommands();

        Component.chassis.setDefaultCommand(
            Component.chassis.c_input(driver::getTranslation, driver::getTurnSpeed)
        );
        // END TODO ^

        // Component.lights.flashColor(LightSubsystem.Color.ENABLED);
    }

    @Override
    public void teleopExecute() {

    }

    @Override
    public void autonomousInitialize() {
        if (!AutonConfig.ENABLED) return;

        // PATHPLANNER ATTEMPT #1520367
        // try {
        //     // Load the path you want to follow using its name in the GUI
        //     PathPlannerPath path = PathPlannerPath.fromPathFile("straight");
        //     AutoBuilder.followPath(path).schedule();
        // } catch (Exception e) {
        //     System.out.println(e);
        // }

        // Component.navx.reset();

        CmdUtil.schedule(AutonConfig.COMMAND.get());
    }

    @Override
    public void autonomousExecute() {
        // logging can go here
    }

    @Override
    public void disabledInitialize() {
        Component.vision.stopPositioning("Robot disabled", false);

        Component.chassis.setMotorBrake(false);
        // Component.lights.flashColor(LightSubsystem.Color.DISABLED);
     }

    @Override
    public void disabledExecute() {
    }

    @Override
    public void testInitialize() {

    }

    @Override
    public void testExecute() {

    }

    double lastLogTime;

    @Override
    public void alwaysExecute() {
        double now = Timer.getFPGATimestamp();
        if (now - lastLogTime >= 1) {
            lastLogTime = now;
            List<Tag> tags = Component.vision.gtm.getTags();
            if (!tags.isEmpty()) System.out.println("WE FOUND A TAG: " + tags);
        }
    }
}
