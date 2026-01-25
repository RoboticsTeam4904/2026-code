package org.usfirst.frc4904.robot.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.usfirst.frc4904.robot.RobotMap.Component;
import org.usfirst.frc4904.standard.commands.NoOp;

import java.util.Arrays;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

final class SwerveConstants {

    // TODO: get real measurements
    static final double DRIVE_GEAR_RATIO = 5.1; // motor rots/wheel rots
    static final double ROT_GEAR_RATIO = 5.1; // motor rots/wheel rots

    static final double ROBOT_DIAGONAL = 1.15; // m
    static final double WHEEL_RADIUS = 0.07; // m

    static final double WHEEL_CIRC = 2 * Math.PI * WHEEL_RADIUS; // m or m/rot

    static final double ROBOT_TURN_CIRC = Math.PI * ROBOT_DIAGONAL; // m

    // TODO: tune
    static final double LIN_SPEED = 6; // m/s
    static final double ROT_SPEED = LIN_SPEED * ROBOT_TURN_CIRC; // rot/s

    private SwerveConstants() {}

}

public class SwerveSubsystem extends SubsystemBase implements Sendable {

    private final SwerveDriveKinematics kinematics;
    private final SwerveDrivePoseEstimator estimator;
    private final SwerveModule[] modules;

    public SwerveSubsystem(SwerveModule... modules) {
        this.modules = modules;

        kinematics = new SwerveDriveKinematics(
            Arrays.stream(modules)
                  .map(module -> module.position)
                  .toArray(Translation2d[]::new)
        );
        estimator = new SwerveDrivePoseEstimator(
            kinematics,
            Component.navx.getRotation2d(),
            getModulePositions(),
            Pose2d.kZero // TODO
        );

        SmartDashboard.putData("swerve/goal", this);
    }

    private SwerveModulePosition[] getModulePositions() {
        return Arrays.stream(modules)
                     .map(SwerveModule::getModulePosition)
                     .toArray(SwerveModulePosition[]::new);
    }

    /**
     * Drive according to joystick inputs. {@code hypot(x, y)} should not exceed 1.
     * @param translation X/Y movement from [-1, 1]
     * @param theta Turn speed from [-1, 1]
     */
    public void input(Translation2d translation, double theta) {
        Translation2d scaled = translation.times(SwerveConstants.LIN_SPEED);
        driveFieldRelative(scaled, theta * SwerveConstants.ROT_SPEED);
    }

    public Translation2d toRobotRelative(Translation2d translation) {
        double rotation = Component.navx.getYaw() + 90;
        return translation.rotateBy(Rotation2d.fromDegrees(-rotation));
    }

    /**
     * Drive relative to the field.
     * @param translation Movement speed in meters per second
     * @param theta Rotation speed in rotations per second - not field-relative,
     *              as it represents the turning speed, not an absolute angle.
     */
    public void driveFieldRelative(Translation2d translation, double theta) {
        driveRobotRelative(toRobotRelative(translation), theta);
    }

    /**
     * Drive relative to the current angle of the robot.
     * @param translation Movement speed in meters per second
     * @param theta Rotation speed in rotations per second
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
        return -Units.degreesToRotations(Component.navx.getYaw());
    }

    /// COMMANDS

    private static final PIDController rotPID = new PIDController(1, 0, 0);
    static {
        rotPID.enableContinuousInput(0, 1);
    }
    private RotateToCommand rotCommand; // non-null when a rot command is running
    double rotPIDEffort;

    public Command c_rotateTo(double theta) {
        return c_rotateTo(() -> theta);
    }

    public Command c_rotateTo(DoubleSupplier getTheta) {
        return new RotateToCommand(getTheta);
    }

    private class RotateToCommand extends Command {

        private final DoubleSupplier getTheta;

        RotateToCommand(DoubleSupplier getTheta) {
            this.getTheta = getTheta;
            // don't require swerve subsystem so that it can run in parallel to other swerve commands
        }

        @Override
        public void initialize() {
            // manually cancel any other active rotate command
            if (rotCommand != null) rotCommand.cancel();
            rotCommand = this;
        }

        @Override
        public void execute() {
            double current = getHeading();
            double goal = getTheta.getAsDouble();
            rotPIDEffort = rotPID.calculate(current, goal);
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

    public void resetOdometry() {
        Component.navx.zeroYaw();
    }

    /**
     * Zero the rotation encoders for all swerve modules.
     */
    public void zero() {
        System.out.println("zeroed");
        for (var module : modules) module.zero();
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
        for (var module : modules) {
            module.addSendableProps(builder);
        }
        builder.addDoubleProperty("Robot Angle", () -> Units.degreesToRotations(Component.navx.getYaw()), null);
    }
}
