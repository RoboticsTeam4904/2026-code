package org.usfirst.frc4904.robot.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.robot.RobotMap.Component;
import org.usfirst.frc4904.robot.vision.GoogleTagManager;
import org.usfirst.frc4904.robot.vision.GoogleTagManager.Tag;
import org.usfirst.frc4904.standard.commands.NoOp;
import org.usfirst.frc4904.standard.util.Logging;
import org.usfirst.frc4904.standard.util.Util;

import java.util.Arrays;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

final class SwerveConstants {
    private SwerveConstants() {}

    static final double DRIVE_GEAR_RATIO = 5.294; // motor rots/wheel rots

    // TODO: get real measurements
    static final double DRIVE_RPM = 6380;

    // TODO: get real measurements
    static final double WHEEL_RADIUS = 0.07; // meters
    static final double ROBOT_DIAGONAL = 1.15; // meters

    static final double WHEEL_CIRC = 2 * Math.PI * WHEEL_RADIUS; // meters

    // m/s
    static final double LIN_SPEED = DRIVE_RPM / 60.0 / DRIVE_GEAR_RATIO * WHEEL_CIRC;
    // turns/s
    static final double ROT_SPEED = LIN_SPEED / (Math.PI * ROBOT_DIAGONAL);
}

public class SwerveSubsystem extends SubsystemBase {

    private final SwerveModule[] modules;

    private final Field2d field = new Field2d();
    private final SwerveDriveKinematics kinematics;
    private final SwerveDrivePoseEstimator estimator;
    private boolean estimatorEnabled = false;

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
            Pose2d.kZero // unused
        );

        SmartDashboard.putData("swerve/goal", this);
        SmartDashboard.putData("swerve/field", field);
    }

    private SwerveModulePosition[] getModulePositions() {
        return Arrays.stream(modules)
                     .map(SwerveModule::getModulePosition)
                     .toArray(SwerveModulePosition[]::new);
    }

    private SwerveModuleState[] getModuleStates() {
        return Arrays.stream(modules)
                     .map(SwerveModule::getModuleState)
                     .toArray(SwerveModuleState[]::new);
    }

    public void startPoseEstimator(Pose2d currentPose) {
        estimator.resetPose(currentPose);
        estimatorEnabled = true;
    }

    public void stopPoseEstimator() {
        estimatorEnabled = false;
    }

    /**
     * Add a vision measurement to the pose estimator.
     * @param pose Pose of the robot (according to the tag measurement)
     * @param time Time of the vision measurement, from {@link Timer#getFPGATimestamp()}
     */
    public void addVisionPoseEstimate(Pose2d pose, double time) {
        if (!estimatorEnabled) {
            System.err.println("SwerveSubsystem.addVisionPoseEstimate() called while pose estimator is disabled.");
            return;
        }

        estimator.addVisionMeasurement(pose, time);
    }

    /**
     * @return The current pose estimate, based on swerve odometry and {@code addVisionPoseEstimate()} calls
     */
    public Pose2d getPoseEstimate() {
        if (!estimatorEnabled) {
            System.err.println("SwerveSubsystem.getPoseEstimate() called while pose estimator is disabled.");
            return Pose2d.kZero;
        }

        return estimator.getEstimatedPosition();
    }

    /**
     * @return current robot-relative {@link ChassisSpeeds} (velocity and direction) according to encoders
     */
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
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
     * @param speeds Target velocity and rotation, deconstructed into a translation and rotation.
     *               See {@link #driveRobotRelative(Translation2d, double)}
     */
    public void driveRobotRelative(ChassisSpeeds speeds) {
        driveRobotRelative(
            new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond),
            Units.radiansToRotations(speeds.omegaRadiansPerSecond)
        );
    }

    /**
     * Drive relative to the current angle of the robot.
     * @param translation Movement speed in meters per second
     * @param theta Rotation speed in rotations per second.
     *              Will be overridden if a {@link #c_rotateTo(double) c_rotateTo()} command is active
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

    private double lastTagUpdateTime;

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) return;

        for (var module : modules) module.periodic();

        if (estimatorEnabled) {
            estimator.update(Component.navx.getRotation2d(), getModulePositions());

            var tags = GoogleTagManager.getTagsSince(lastTagUpdateTime);
            lastTagUpdateTime = GoogleTagManager.getLastTime();
            
            for (var tag : tags) {
                if (tag.id() == 0) { // TODO temporary
                    // all field-relative
                    Rotation2d heading = Rotation2d.fromRotations(getHeading());
                    Translation2d robotToTag = tag.pos().toTranslation2d().rotateBy(heading);
                    Translation2d robotPos = tag.fieldPos().getTranslation().minus(robotToTag);
                    
                    Logging.log("WE HAVE A POS", robotPos);

                    addVisionPoseEstimate(
                        new Pose2d(robotPos, heading),
                        Timer.getFPGATimestamp() // TODO undo
                        // tag.time()
                    );
                }
            }

            field.setRobotPose(getPoseEstimate());
        }
    }

    public double getHeading() {
        return Component.navx.getYaw();
    }

    /// COMMANDS

    private RotateCommand rotCommand; // non-null when a rot command is running
    private double rotPIDEffort;

    /**
     * @param theta Field-relative angle to rotate to
     * @return A command that uses PID to rotate to the provided angle.
     *         Overrides any rotation from any other drive commands or methods while the command is running
     */
    public Command c_rotateTo(double theta) {
        return c_rotateTo(() -> theta);
    }

    /**
     * @param getTheta Supplier of field-relative angles to rotate to
     * @return A command that uses PID to rotate to the provided angle.
     *         Overrides any rotation from any other drive commands or methods while the command is running
     */
    public Command c_rotateTo(DoubleSupplier getTheta) {
        return new RotateCommand(getTheta, true);
    }

    /**
     * @param getTheta Supplies doubles representing the difference between the current heading and target angle
     * @return A command that uses PID to rotate to the provided angle.
     *         Overrides any rotation from any other drive commands or methods while the command is running
     */
    public Command c_controlRotation(DoubleSupplier getTheta) {
        return new RotateCommand(getTheta, false);
    }

    private class RotateCommand extends Command {

        private final PIDController rotPID;
        private final DoubleSupplier getTheta;
        private final boolean fieldRelative;

        RotateCommand(DoubleSupplier getTheta, boolean fieldRelative) {
            this.getTheta = getTheta;
            this.fieldRelative = fieldRelative;

            rotPID = new PIDController(20, 0, 0);
            rotPID.enableContinuousInput(0, 1);
            // don't require swerve subsystem so that it can run in parallel to other swerve commands
        }

        @Override
        public void initialize() {
            // manually cancel any other active rotate command
            if (rotCommand != null) rotCommand.cancel();
            rotCommand = this;
            rotPID.reset();
        }

        @Override
        public void execute() {
            double current = fieldRelative ? getHeading() : 0;
            double goal = getTheta.getAsDouble();

            rotPIDEffort = Util.clamp(
                rotPID.calculate(current, goal),
                -SwerveConstants.ROT_SPEED,
                SwerveConstants.ROT_SPEED
            );
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
    public Command c_input(Supplier<? extends Translation2d> translation, DoubleSupplier theta) {
        return run(() -> input(translation.get(), theta.getAsDouble()));
    }

    /// MISC CONFIG

    public void resetOdometry() {
        Component.navx.zeroYaw();
        estimator.resetRotation(Rotation2d.kZero);
    }

    /**
     * Zero the rotation encoders for all swerve modules.
     */
    public void zero() {
        System.out.println("zeroed");
        for (var module : modules) module.zero();
    }

    /**
     * Flip current zero position by 180deg.
     */
    public void flipZero() {
        System.out.println("flipped zero");
        for (var module : modules) module.flipZero();
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
