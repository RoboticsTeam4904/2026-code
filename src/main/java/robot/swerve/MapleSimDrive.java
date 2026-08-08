package robot.swerve;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import lib.custom.motorcontrollers.CustomTalonFX;
import lib.custom.sensors.CustomDutyCycleEncoder;
import org.littletonrobotics.junction.Logger;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.simulation.motorsims.SimulatedMotorController.GenericMotorController;

public class MapleSimDrive {
    private static final double ROBOT_MASS = 55;
    private static final double BUMPER_X = 0.84;
    private static final double BUMPER_Y = 0.84;
    private static final double TRACK_X = SwerveConstants.ROBOT_DIAGONAL;
    private static final double TRACK_Y = SwerveConstants.ROBOT_DIAGONAL;
    private static final double DRIVE_GEAR_RATIO = 5.625;
    private static final double STEER_GEAR_RATIO = 12.8;
    private static final double DRIVE_FRICTION_VOLTAGE = 0.05;
    private static final double STEER_FRICTION_VOLTAGE = 0.15;
    private static final double STEER_INERTIA = 0.03;
    private static final double COF = 1.2;

    private final SwerveDriveSimulation driveSim;
    private final CustomTalonFX[] driveMotors;
    private final CustomTalonFX[] turnMotors;
    private final CustomDutyCycleEncoder[] turnEncoders;
    private final Pigeon2 pigeon;
    private final GenericMotorController[] driveControllers;
    private final GenericMotorController[] steerControllers;
    private final robot.simulation.FuelSim fuelSimulation;

    public MapleSimDrive(
        CustomTalonFX[] driveMotors,
        CustomTalonFX[] turnMotors,
        CustomDutyCycleEncoder[] turnEncoders,
        Pigeon2 pigeon
    ) {
        this.driveMotors = driveMotors;
        this.turnMotors = turnMotors;
        this.turnEncoders = turnEncoders;
        this.pigeon = pigeon;

        Translation2d[] translations = {
            new Translation2d(TRACK_X / 2, TRACK_Y / 2),
            new Translation2d(TRACK_X / 2, -TRACK_Y / 2),
            new Translation2d(-TRACK_X / 2, TRACK_Y / 2),
            new Translation2d(-TRACK_X / 2, -TRACK_Y / 2)
        };

        SwerveModuleSimulationConfig moduleConfig = new SwerveModuleSimulationConfig(
            DCMotor.getKrakenX60(1),
            DCMotor.getKrakenX44(1),
            DRIVE_GEAR_RATIO,
            STEER_GEAR_RATIO,
            Volts.of(DRIVE_FRICTION_VOLTAGE),
            Volts.of(STEER_FRICTION_VOLTAGE),
            Meters.of(SwerveConstants.WHEEL_DIAMETER / 2),
            KilogramSquareMeters.of(STEER_INERTIA),
            COF
        );

        DriveTrainSimulationConfig config = new DriveTrainSimulationConfig(
            Kilograms.of(ROBOT_MASS),
            Meters.of(BUMPER_X),
            Meters.of(BUMPER_Y),
            Meters.of(TRACK_X),
            Meters.of(TRACK_Y),
            COTS.ofPigeon2(),
            () -> new SwerveModuleSimulation(moduleConfig)
        ).withCustomModuleTranslations(translations);

        driveSim = new SwerveDriveSimulation(config, new Pose2d(7.0, 1.72, new Rotation2d(Math.PI)));

        SwerveModuleSimulation[] modules = driveSim.getModules();
        driveControllers = new GenericMotorController[4];
        steerControllers = new GenericMotorController[4];
        for (int i = 0; i < 4; i++) {
            driveControllers[i] = modules[i].useGenericMotorControllerForDrive().withCurrentLimit(Amps.of(40));
            steerControllers[i] = modules[i].useGenericControllerForSteer().withCurrentLimit(Amps.of(20));
        }

        pigeon.getSimState().setRawYaw(0);
        for (var encoder : turnEncoders) {
            new DutyCycleEncoderSim(encoder).set((encoder.getResetOffset() + 1) % 1);
        }

        SimulatedArena.getInstance().addDriveTrainSimulation(driveSim);

        fuelSimulation = new robot.simulation.FuelSim(driveSim);
    }

    public void setSimulationWorldPose(Pose2d pose) {
        driveSim.setSimulationWorldPose(pose);
    }

    public Pose2d getSimulationWorldPose() {
        return driveSim.getSimulatedDriveTrainPose();
    }

    public void periodic() {
        for (int i = 0; i < 4; i++) {
            driveControllers[i].requestVoltage(Volts.of(driveMotors[i].getSimState().getMotorVoltage()));
            steerControllers[i].requestVoltage(Volts.of(turnMotors[i].getSimState().getMotorVoltage()));
        }

        SimulatedArena.getInstance().simulationPeriodic();
        Logger.recordOutput("SimPhysics/DriveTrainPose", driveSim.getSimulatedDriveTrainPose());
        fuelSimulation.periodic();

        SwerveModuleSimulation[] modules = driveSim.getModules();
        for (int i = 0; i < 4; i++) {
            SwerveModuleSimulation module = modules[i];

            double wheelRotations = module.getDriveWheelFinalPosition().in(Rotations);
            driveMotors[i].getSimState().setRawRotorPosition(Rotations.of(wheelRotations * SwerveConstants.DRIVE_GEAR_RATIO));
            driveMotors[i].getSimState().setRotorVelocity(
                RadiansPerSecond.of(module.getDriveWheelFinalSpeed().in(RadiansPerSecond) * SwerveConstants.DRIVE_GEAR_RATIO)
            );

            double facingRotations = module.getSteerAbsoluteFacing().getRotations();
            double raw = (turnEncoders[i].getResetOffset() - facingRotations) % 1;
            if (raw < 0) raw += 1;
            new DutyCycleEncoderSim(turnEncoders[i]).set(raw);
        }

        pigeon.getSimState().setRawYaw(
            Units.radiansToDegrees(driveSim.getGyroSimulation().getGyroReading().getRadians())
        
            );
    }
}