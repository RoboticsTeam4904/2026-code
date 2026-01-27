package org.usfirst.frc4904.robot.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.usfirst.frc4904.robot.RobotMap.Component;
import org.usfirst.frc4904.standard.commands.NoOp;
import org.usfirst.frc4904.standard.util.Util;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

final class SwerveConstants {
    private SwerveConstants() {}

    // TODO: get real measurements
    public static final double LIN_RPM = 6380;
    public static final double LIN_GEAR_RATIO = 5.1;

    // meters - TODO: get real measurements
    public static final double WHEEL_RADIUS = 0.07;
    public static final double ROBOT_DIAGONAL = 1.15;

    // m/s
    public static final double LIN_SPEED = LIN_RPM / 60.0 / LIN_GEAR_RATIO * (2 * Math.PI * WHEEL_RADIUS);
    // turns/s
    public static final double ROT_SPEED = LIN_SPEED / (Math.PI * ROBOT_DIAGONAL);
}

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule[] modules;

    public SwerveSubsystem(SwerveModule... modules) {
        this.modules = modules;

        SmartDashboard.putData("swerve/goal", this);
    }

    /**
     * Drive according to joystick inputs. {@code hypot(x, y)} should not exceed 1.
     * @param translation X/Y movement from [-1, 1], wpilib coordinate system (forward, left)
     * @param theta Turn speed from [-1, 1], positive = counterclockwise
     */
    public void input(Translation2d translation, double theta) {
        Translation2d scaled = translation.times(SwerveConstants.LIN_SPEED);
        driveFieldRelative(scaled, theta * SwerveConstants.ROT_SPEED);
    }

    public Translation2d toRobotRelative(Translation2d translation) {
        return translation.rotateBy(Rotation2d.fromRotations(-getHeading()));
    }

    /**
     * Drive relative to the field.
     * @param translation Movement speed in meters per second
     * @param theta Rotation speed in rotations per second - not field-relative,
     *              as it represents the turning speed, not an absolute angle.
     *              Will be overridden if a c_rotateTo() command is active
     */
    public void driveFieldRelative(Translation2d translation, double theta) {
        driveRobotRelative(toRobotRelative(translation), theta);
    }

    /**
     * Drive relative to the current angle of the robot.
     * @param translation Movement speed in meters per second
     * @param theta Rotation speed in rotations per second. Will be overridden if a c_rotateTo() command is active
     */
    public void driveRobotRelative(Translation2d translation, double theta) {
        if (rotCommand != null) {
            theta = rotPIDEffort;
        }

        Translation2d[] translations = new Translation2d[modules.length];
        double maxMag = SwerveConstants.LIN_SPEED;

        for (int i = 0; i < modules.length; i++) {
            Translation2d rotation = modules[i].rotToTranslation(theta);
            Translation2d sum = translation.plus(rotation);

            translations[i] = sum;
            maxMag = Math.max(sum.getNorm(), maxMag);
        }

        double norm = maxMag / SwerveConstants.LIN_SPEED;

        for (int i = 0; i < modules.length; i++) {
            Translation2d normalized = translations[i].div(norm);

            double magnitude = normalized.getNorm();
            modules[i].moveTo(
                magnitude,
                magnitude > 0 ? normalized.getAngle().getRotations() : 0
            );
        }
    }

    /** See {@link #driveRobotRelative(Translation2d, double)} */
    public void driveRobotRelative(double x, double y, double theta) {
        driveRobotRelative(new Translation2d(x, y), theta);
    }

    public void stop() {
        driveRobotRelative(0, 0, 0);
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) return;
        for (var module : modules) module.periodic();
    }

    double getHeading() {
        return Component.navx.getYaw();
    }

    /// COMMANDS

    private RotateToCommand rotCommand; // non-null when a rot command is running
    private double rotPIDEffort;

    public Command c_rotateTo(double theta) {
        return c_rotateTo(() -> theta);
    }

    public Command c_rotateTo(Supplier<Double> getTheta) {
        return new RotateToCommand(getTheta);
    }

    private class RotateToCommand extends Command {

        private final PIDController rotPID;
        private final Supplier<Double> getTheta;

        RotateToCommand(Supplier<Double> getTheta) {
            this.getTheta = getTheta;

            rotPID = new PIDController(20, 0, 0);
            rotPID.enableContinuousInput(0, 1);
            // don't require swerve subsystem so that it can run in parallel to other swerve commands
        }

        @Override
        public void initialize() {
            // manually cancel any other active rotate command
            if (rotCommand != null) rotCommand.cancel();
            rotCommand = this;
        }

        private Double lastGoal;

        @Override
        public void execute() {
            double current = getHeading();
            Double nextGoal = getTheta.get();

            if (nextGoal == null && lastGoal == null) {
                rotPIDEffort = 0;
            } else {
                double goal = nextGoal == null ? lastGoal : nextGoal;
                lastGoal = goal;
                System.out.println("goal: " + goal);
                rotPIDEffort = Util.clamp(
                    rotPID.calculate(0, goal), // TODO change back to current
                    -SwerveConstants.ROT_SPEED,
                    SwerveConstants.ROT_SPEED
                );
            }
        }

        @Override
        public void end(boolean interrupted) {
            rotCommand = null;
        }
    }

    /**
     * Stop the wheels.
     */
    public Command c_stop() {
        return runOnce(this::stop);
    }

    /**
     * Hold a field-relative movement speed and rotation speed.
     * <p>
     * See {@link #driveFieldRelative(Translation2d, double)}
     */
    public Command c_driveFieldRelative(Translation2d translation, double theta) {
        return run(() -> driveFieldRelative(translation, theta));
    }

    /**
     * Hold a field-relative movement speed and rotation speed.
     * <p>
     * See {@link #driveFieldRelative(Translation2d, double)}
     */
    public Command c_driveFieldRelative(double x, double y, double theta) {
        Translation2d translation = new Translation2d(x, y);
        return run(() -> driveFieldRelative(translation, theta));
    }

    /**
     * Hold a robot-relative movement speed and rotation speed.
     * <p>
     * See {@link #driveRobotRelative(Translation2d, double)}
     */
    public Command c_driveRobotRelative(Translation2d translation, double theta) {
        return run(() -> driveRobotRelative(translation, theta));
    }

    /**
     * Hold a robot-relative movement speed and rotation speed.
     * <p>
     * See {@link #driveRobotRelative(Translation2d, double)}
     */
    public Command c_driveRobotRelative(double x, double y, double theta) {
        Translation2d translation = new Translation2d(x, y);
        return run(() -> driveRobotRelative(translation, theta));
    }

    /**
     * Drive according to inputs provided by the suppliers.
     * <p>
     * See {@link #input(Translation2d, double)}
     */
    public Command c_input(Supplier<Translation2d> translation, DoubleSupplier theta) {
        return run(() -> input(translation.get(), theta.getAsDouble()));
    }

    /// MISC CONFIG

    public void resetOdometry() {
        Component.navx.zeroYaw();
    }

    public void setMotorBrake(boolean brake) {
        for (var module : modules) module.setMotorBrake(brake);
    }

    public Command getAutonomousCommand(String path, boolean setOdom, boolean flipSide) {
        return new NoOp(); // TODO
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");
        builder.addDoubleProperty("Robot Angle", this::getHeading, null);

        for (var module : modules) module.addSendableProps(builder);
    }
}
