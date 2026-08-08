package robot.simulation;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import java.util.ArrayList;
import java.util.List;
import lib.custom.motorcontrollers.CustomTalonFX;
import org.littletonrobotics.junction.Logger;
import robot.RobotMap.Component;
import robot.subsystems.IntakeSubsystem;

public class MechanismSim {
    public static final double INTAKE_INERTIA = 0.0005; //tuned until mechanism feels good, probably not correct way to do it but idrc
    public static final double INTAKE_ROTOR_PER_ENCODER_ROT = 80;
    public static final double CLIMBER_INERTIA = 0.001;
    public static final double CLIMBER_ROTOR_PER_ENCODER_ROT = 50;
    private static final double LOOP_DT = 0.02;
    public static final double INTAKE_PIVOT_X = 0, INTAKE_PIVOT_Y = 0.2, INTAKE_PIVOT_Z = 0.2;
    public static final double CLIMBER_BASE_X = 0, CLIMBER_BASE_Y = 0, CLIMBER_BASE_Z = 0;
    public static final double CLIMBER_M_PER_ENCODER_ROT = 0.2 / 1.87; 

    private final List<SimMotor> sims = new ArrayList<>();

    public void addEncoderMotor(
        CustomTalonFX motor,
        DutyCycleEncoder encoder,
        double rotorPerEncoderRot,
        double inertia,
        double initialEncoderRot
    ) {
        sims.add(new SimMotor(motor, inertia, encoder, rotorPerEncoderRot, initialEncoderRot));
    }

    public void periodic() {
        for (SimMotor sim : sims) sim.update(LOOP_DT);
        double armAngle = (Component.intake.getAngle() - IntakeSubsystem.EXTEND_ANGLE)/IntakeSubsystem.ENCODER_RATIO * 2 * Math.PI;
        double climberHeight = Component.climber.getHeight() * CLIMBER_M_PER_ENCODER_ROT;
        Logger.recordOutput("Mechanism/ComponentPoses", new Pose3d[] {
            new Pose3d(new Translation3d(INTAKE_PIVOT_X, INTAKE_PIVOT_Y, INTAKE_PIVOT_Z), new Rotation3d(armAngle, 0, 0)),
            new Pose3d(
                new Translation3d(CLIMBER_BASE_X, CLIMBER_BASE_Y, CLIMBER_BASE_Z + climberHeight),
                new Rotation3d()
            ),
        });
    }

    private static class SimMotor {

        private final TalonFXSimState motorSim;
        private final DutyCycleEncoderSim encoder;
        private final DCMotorSim plant;
        private final double rotorPerEncoderRot;
        private final double initialEncoderRot;
        private final boolean inverted;

        SimMotor(
            CustomTalonFX motor,
            double inertia,
            DutyCycleEncoder encoder,
            double rotorPerEncoderRot,
            double initialEncoderRot
        ) {
            motorSim = motor.getSimState();
            this.encoder = encoder == null ? null : new DutyCycleEncoderSim(encoder);
            this.rotorPerEncoderRot = rotorPerEncoderRot;
            this.initialEncoderRot = initialEncoderRot;
            this.inverted = motor.getInverted();
            plant = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), inertia, 1),
                DCMotor.getKrakenX60(1)
            );
        }

        void update(double dt) {
            plant.setInputVoltage(motorSim.getMotorVoltage());
            plant.update(dt);

            double rotorRot = plant.getAngularPositionRotations();
            motorSim.setRawRotorPosition(Rotations.of(rotorRot));
            motorSim.setRotorVelocity(RadiansPerSecond.of(plant.getAngularVelocityRadPerSec()));

            if (encoder != null) {
                double encoderRot = initialEncoderRot + rotorRot / rotorPerEncoderRot * (inverted ? -1 : 1);
                encoder.set(encoderRot % 1);
            }
        }
    }
}