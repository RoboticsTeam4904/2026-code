package org.usfirst.frc4904.robot;

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
import org.usfirst.frc4904.standard.custom.motorcontrollers.CustomTalonFX;
import org.usfirst.frc4904.standard.custom.sensors.CustomDutyCycleEncoder;

public class RobotMap {

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

        Component.chassis = new SwerveSubsystem(
            new SwerveModule(
                "Front Left",
                new CustomTalonFX(2),
                new CustomTalonFX(17),
                new CustomDutyCycleEncoder(Port.PWM.ENCODER_FL),
                new Translation2d(1, 1)
            ),
            new SwerveModule(
                "Front Right",
                new CustomTalonFX(3),
                new CustomTalonFX(15),
                new CustomDutyCycleEncoder(Port.PWM.ENCODER_FR),
                new Translation2d(-1, 1)
            ),
            new SwerveModule(
                "Back Left",
                new CustomTalonFX(4),
                new CustomTalonFX(18),
                new CustomDutyCycleEncoder(Port.PWM.ENCODER_BL),
                new Translation2d(1, -1)
            ),
            new SwerveModule(
                "Back Right",
                new CustomTalonFX(1),
                new CustomTalonFX(16),
                new CustomDutyCycleEncoder(Port.PWM.ENCODER_BR),
                new Translation2d(-1, -1)
            )
        );

        Component.vision = new VisionSubsystem(
            new Transform2d(Units.inchesToMeters(8), Units.inchesToMeters(0), Rotation2d.kZero)
        );

        Component.ledStrip = new AddressableLED(Port.PWM.LED_STRIP);
        Component.lights = new LightSubsystem(
            Component.ledStrip,
            107,
            new int[] { 20, 37, 34, 16 },
            new boolean[] { false, true, false, true }
        );

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
}
