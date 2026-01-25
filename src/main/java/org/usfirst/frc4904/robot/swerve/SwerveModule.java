package org.usfirst.frc4904.robot.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc4904.standard.custom.motorcontrollers.SmartMotorController;

public class SwerveModule implements Sendable {

    final String name;
    final Translation2d position;

    private final DriveController drive;
    private final RotationController rotation;

    private double magnitude;
    private double theta;

    public SwerveModule(
        String name,
        SmartMotorController driveMotor,
        SmartMotorController rotMotor,
        Translation2d position
    ) {
        this.name = name;
        // TODO maybe remove normalization and make it the caller's responsibility to pass the position in meters
        this.position = position.times(SwerveConstants.ROBOT_DIAGONAL / (2 * position.getNorm()));

        Translation2d direction = position.rotateBy(Rotation2d.kCCW_90deg);

        drive = driveMotor != null ? new DriveController(driveMotor) : null;
        rotation = new RotationController(rotMotor, direction, name);

        theta = rotation.getRotation();

        SmartDashboard.putData("swerve/" + name, this);
    }

    Translation2d rotToTranslation(double theta) {
        return rotation.toTranslation(theta);
    }

    SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
            drive.getDistance(),
            Rotation2d.fromRotations(rotation.getRotation())
        );
    }

    void zero() {
        rotation.zero();
    }

    void setMotorBrake(boolean brake) {
        if (brake) {
            drive.motor().setBrakeOnNeutral();
            rotation.motor.setBrakeOnNeutral();
        } else {
            drive.motor().setCoastOnNeutral();
            rotation.motor.setCoastOnNeutral();
        }
    }

    void moveTo(double magnitude, double theta) {
        this.magnitude = magnitude;
        if (magnitude > 0) this.theta = theta;
    }

    void periodic() {
        double angleDist = rotation.rotateToward(theta);
        if (drive != null) drive.setMagnitude(magnitude * angleDist);
    }

    /// SMART DASHBOARD

    void addSendableProps(SendableBuilder builder) {
        builder.addDoubleProperty(name + " Angle", () -> theta, null);
        builder.addDoubleProperty(name + " Velocity", () -> magnitude, null);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Swerve Module");
        builder.addDoubleProperty("angle", rotation::getRotation, null);
        builder.addDoubleProperty("delta", () -> {
            double delta = theta - rotation.getRotation();
            return MathUtil.inputModulus(delta, -0.5, 0.5);
        }, null);
        builder.addDoubleProperty("zero", rotation.motor::getMechanismRotationOffset, rotation.motor::setMechanismRotationOffset);
    }
}

record DriveController(SmartMotorController motor) {

    // TODO tune
    private static final double kP = 1, kI = 0, kD = 0;

    DriveController {
        // TODO feedforward?
        motor.setPID(kP, kI, kD)
             .setMotorMechanismRatio(SwerveConstants.DRIVE_GEAR_RATIO);
    }

    double getDistance() {
        return motor.getRotation() * SwerveConstants.WHEEL_CIRC;
    }

    void setMagnitude(double mps) {
        motor.holdVelocity(mps / SwerveConstants.WHEEL_CIRC);
    }
}

class RotationController {

    // TODO tune
    private static final double kP = 15, kI = 0, kD = 0;

    final SmartMotorController motor;

    private final Translation2d direction;
    private final String key;

    RotationController(SmartMotorController motor, Translation2d direction, String name) {
        this.motor = motor;
        this.direction = direction.div(direction.getNorm());

        key = "swerve zeros/" + name;

        // encoder readings are from 0-1 but opposite angles are equivalent
        // since we can just run the wheels backwards
        // TODO feedforward?
        motor.setPID(kP, kI, kD)
             .setContinuousRange(0.5)
             .setMotorMechanismRatio(SwerveConstants.ROT_GEAR_RATIO)
             .setMechanismRotationOffset(Preferences.getDouble(key, 0));
    }

    Translation2d toTranslation(double theta) {
        return direction.times(theta);
    }

    void zero() {
        motor.zeroMechanismRotationOffset();
        Preferences.setDouble(key, motor.getMechanismRotationOffset());
    }

    double getRotation() {
        return motor.getRotation();
    }

    /**
     * @return Similarity between current and target rotation.
     * Effectively a dot product: 1 = same angle, -1 = opposite, 0 = perpendicular.
     */
    double rotateToward(double theta) {
        motor.holdPosition(theta);

        double current = getRotation();
        return Math.cos(Units.rotationsToRadians(theta - current));
    }
}
