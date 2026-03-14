package org.usfirst.frc4904.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class Constants {

    public static class Intake {

        private static final String ROLLER_VOLTS = "Constants/Indexer/RollerVolts";

        private static void init() {
            SmartDashboard.setDefaultNumber(ROLLER_VOLTS, 6);

            SmartDashboard.setPersistent(ROLLER_VOLTS);
        }

        public static final double rollerVolts() {
            return SmartDashboard.getNumber(ROLLER_VOLTS, 6);
        }
    }

    public static class Indexer {

        private static final String BOTH_VOLTS = "Constants/Intake/BothVolts";
        private static final String TOP_MULT = "Constants/Intake/TopMult";
        private static final String BOTTOM_MULT = "Constants/Intake/BottomMult";

        private static final void init() {
            SmartDashboard.setDefaultNumber(BOTH_VOLTS, 5);
            SmartDashboard.setDefaultNumber(TOP_MULT, 1);
            SmartDashboard.setDefaultNumber(BOTTOM_MULT, 1);

            SmartDashboard.setPersistent(BOTH_VOLTS);
            SmartDashboard.setPersistent(TOP_MULT);
            SmartDashboard.setPersistent(BOTTOM_MULT);
        }

        public static final double bothVolts() {
            return SmartDashboard.getNumber(BOTH_VOLTS, 5);
        }

        public static final double topMult() {
            return SmartDashboard.getNumber(TOP_MULT, 5);
        }

        public static final double bottomMult() {
            return SmartDashboard.getNumber(BOTTOM_MULT, 5);
        }
    }

    public static class Shooter {

        private static final String ACCOUNT_FOR_ROBOT_VEL = "Constants/Shooter/AccountForRobotVel";

        // multiplier to get from target fuel velocity to target flywheel velocity
        // in theory, this would be ~2 to account for the rotation of the fuel as it leaves the shooter
        private static final String VELOCITY_MULT = "Constants/Shooter/VelocityMult";

        private static void init() {
            SmartDashboard.setDefaultBoolean(ACCOUNT_FOR_ROBOT_VEL, true);
            SmartDashboard.setDefaultNumber(VELOCITY_MULT, 2.35);

            SmartDashboard.setPersistent(ACCOUNT_FOR_ROBOT_VEL);
            SmartDashboard.setPersistent(VELOCITY_MULT);
        }

        public static boolean accountForRobotVel() {
            return SmartDashboard.getBoolean(ACCOUNT_FOR_ROBOT_VEL, true);
        }

        public static double velocityMult() {
            return SmartDashboard.getNumber(VELOCITY_MULT, 2.35);
        }
    }

    public static void init() {
        Indexer.init();
        Intake.init();
        Shooter.init();
    }
}
