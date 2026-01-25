package org.usfirst.frc4904.robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import org.usfirst.frc4904.robot.subsystems.LightSubsystem;
import org.usfirst.frc4904.robot.swerve.SwerveModule;
import org.usfirst.frc4904.robot.swerve.SwerveSubsystem;
import org.usfirst.frc4904.robot.vision.VisionSubsystem;
import org.usfirst.frc4904.standard.custom.controllers.CustomCommandJoystick;
import org.usfirst.frc4904.standard.custom.controllers.CustomCommandXbox;
import org.usfirst.frc4904.standard.custom.motorcontrollers.CustomSparkMax;
import org.usfirst.frc4904.standard.custom.motorcontrollers.CustomTalonFX;
import org.usfirst.frc4904.standard.custom.sensors.CustomDutyCycleEncoder;

public final class RobotMap {

    public static class Port {

        public static class HumanInput {

            public static final int xyJoystickPort = 0;
            public static final int zJoystickPort = 1;

            public static final int joystick = 2;
        }

        public static class CANMotor {

        }

        public static class PWM {

            public static final int ENCODER_FL = 0;
            public static final int ENCODER_FR = 1;
            public static final int ENCODER_BL = 2;
            public static final int ENCODER_BR = 3;

            public static final int LED_STRIP = 9;
        }
    }

    public static class Component {

        public static AHRS navx;

        // subsystems
        public static SwerveSubsystem chassis;
        public static LightSubsystem lights;
        public static VisionSubsystem vision;

        // motors

        // misc
        public static AddressableLED ledStrip;
    }

    public static class Input {}

    public static class HumanInput {

        public static class Driver {

            public static CustomCommandXbox xbox;
            public static CustomCommandJoystick xyJoystick;
            public static CustomCommandJoystick turnJoystick;
        }

        public static class Operator {

            public static CustomCommandJoystick joystick;
        }
    }

    private static boolean initialized = false;

    public static void initialize() {
        if (initialized) {
            System.err.println("Robot already initialized");
            return;
        }
        initialized = true;

        Component.navx = new AHRS(NavXComType.kMXP_SPI);

        var flTurn = new CustomSparkMax(5,MotorType.kBrushless);
        var frTurn = new CustomSparkMax(6,MotorType.kBrushless);
        var blTurn = new CustomSparkMax(7,MotorType.kBrushless);
        var brTurn = new CustomSparkMax(8,MotorType.kBrushless);


        Component.chassis = new SwerveSubsystem(
            new SwerveModule(
                new CustomTalonFX(1),
                flTurn,
                flTurn.getAbsoluteEncoder(),
                new Translation2d(1, 1)
            ),
            new SwerveModule(
                new CustomTalonFX(2),
                frTurn,
                frTurn.getAbsoluteEncoder(),
                new Translation2d(-1, 1)
            ),
            new SwerveModule(
                new CustomTalonFX(3),
                blTurn,
                blTurn.getAbsoluteEncoder(),
                new Translation2d(1, -1)
            ),
            new SwerveModule(
                new CustomTalonFX(4),
                brTurn,
                brTurn.getAbsoluteEncoder(),
                new Translation2d(-1, -1)
            )
        );

        Component.vision = new VisionSubsystem(
            new Transform2d(Units.inchesToMeters(8), Units.inchesToMeters(0), Rotation2d.kZero)
        );

        // Component.ledStrip = new AddressableLED(Port.PWM.LED_STRIP);
        // Component.lights = new LightSubsystem(
        //     Component.ledStrip,
        //     107,
        //     new int[] { 20, 37, 34, 16 },
        //     new boolean[] { false, true, false, true }
        // );

        HumanInput.Driver.xyJoystick = new CustomCommandJoystick(
            Port.HumanInput.xyJoystickPort,
            0.0
        );
        HumanInput.Driver.turnJoystick = new CustomCommandJoystick(
            Port.HumanInput.zJoystickPort,
            0.0
        );

        HumanInput.Operator.joystick = new CustomCommandJoystick(Port.HumanInput.joystick, 0.01);
    }

    private RobotMap() {}

}
