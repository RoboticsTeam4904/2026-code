package org.usfirst.frc4904.robot.humaninterface.drivers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import org.usfirst.frc4904.robot.RobotMap.Component;
import org.usfirst.frc4904.robot.RobotMap.HumanInput;
import org.usfirst.frc4904.standard.humaninput.Driver;

import static org.usfirst.frc4904.robot.humaninterface.HumanInterfaceConfig.JOYSTICK_DEADZONE;

public class SwerveGain extends Driver {

    private static final double SPEED_EXP = 2, TURN_EXP = 2; // TODO TUNE

    public SwerveGain() {
        super("SwerveGain");
    }

    protected double scaleGain(double input, double exp) {
        return MathUtil.copyDirectionPow(input, exp);
    }

    @Override
    public void bindCommands() {
        Component.chassis.setDefaultCommand(
            Component.chassis.c_input(this::getTranslation, this::getTurnSpeed)
                .withName("Driver - swerve drive")
        );

        // RobotMap.HumanInput.Driver.turnJoystick.button1.onTrue(
        //     new InstantCommand(() -> RobotMap.Component.chassis.brickMode())
        // );
        // RobotMap.HumanInput.Driver.turnJoystick.button2.onTrue(
        //     new InstantCommand(() -> RobotMap.Component.chassis.zeroGyro())
        // );
    }

    @Override
    public void unbindCommands() {
        Component.chassis.removeDefaultCommand();
    }

    protected double getRawForward() {
        return -HumanInput.Driver.xyJoystick.getY();
    }
    protected double getRawLeft() {
        return -HumanInput.Driver.xyJoystick.getX();
    }

    @Override
    public Translation2d getTranslation() {
        Translation2d translation = new Translation2d(getRawForward(), getRawLeft());
        double mag = translation.getNorm();
        if (mag == 0) return translation;

        double len = scaleGain(MathUtil.applyDeadband(mag, JOYSTICK_DEADZONE), SPEED_EXP);
        return translation.times(len / mag); // unit translation * len
    }

    @Override
    public double getTurnSpeed() {
        double turnSpeed = -HumanInput.Driver.turnJoystick.getX();
        return scaleGain(turnSpeed, TURN_EXP);
    }
}
