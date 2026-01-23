package org.usfirst.frc4904.robot.humaninterface.drivers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.usfirst.frc4904.robot.RobotMap;
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

    public void bindCommands() {
        Component.chassis.setDefaultCommand(
            Component.chassis.c_input(this::getTranslation, this::getTurnSpeed)
        );

        // RobotMap.HumanInput.Driver.turnJoystick.button1.onTrue(
        //     new InstantCommand(() -> RobotMap.Component.chassis.brickMode())
        // );
        // RobotMap.HumanInput.Driver.turnJoystick.button2.onTrue(
        //     new InstantCommand(() -> RobotMap.Component.chassis.zeroGyro())
        // );
    }

    public void unbindCommands() {
        Component.chassis.removeDefaultCommand();
    }

    // positive x = right
    public double getX() {
        double raw = HumanInput.Driver.xyJoystick.getX();
        return scaleGain(raw, SPEED_EXP);
    }

    // positive y = down
    public double getY() {
        double raw = HumanInput.Driver.xyJoystick.getY();
        return scaleGain(raw, SPEED_EXP);
    }

    // returned translation is in standard field-relative coordinates (forward, left)
    public Translation2d getTranslation() {
        double forward = -HumanInput.Driver.xyJoystick.getY();
        double left    = -HumanInput.Driver.xyJoystick.getX();

        Vector<N2> vec = MathUtil.applyDeadband(VecBuilder.fill(left, forward), JOYSTICK_DEADZONE);
        return new Translation2d(vec);
    }

    public double getTurnSpeed() {
        double raw = HumanInput.Driver.turnJoystick.getX();
        return scaleGain(raw, TURN_EXP);
    }
}
