package robot.simulation;

import static edu.wpi.first.units.Units.Meters;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import robot.RobotMap.Component;


public class FuelSim {

    private static final double INTAKE_WIDTH = 0.7;
    private static final double INTAKE_EXTENSION = 0.2;
    private static final IntakeSimulation.IntakeSide INTAKE_SIDE = IntakeSimulation.IntakeSide.LEFT;
    private static final int INTAKE_CAPACITY = 30;
    private static final double ROLLER_RUNNING_VOLTAGE = 0.5;

    private final IntakeSimulation intake;

    public FuelSim(SwerveDriveSimulation driveSim) {
        SimulatedArena.getInstance().resetFieldForAuto();
        intake = IntakeSimulation.OverTheBumperIntake(
            "Fuel",
            driveSim,
            Meters.of(INTAKE_WIDTH),
            Meters.of(INTAKE_EXTENSION),
            INTAKE_SIDE,
            INTAKE_CAPACITY
        );
    }

    public void periodic() {
        double rollerVoltage =
            ((lib.custom.motorcontrollers.CustomTalonFX) Component.intakeRollerMotor).getSimState().getMotorVoltage();
        if (Math.abs(rollerVoltage) > ROLLER_RUNNING_VOLTAGE) {
            intake.startIntake();
        } else {
            intake.stopIntake();
        }

        Logger.recordOutput("FieldSimulation/ResetField", resetField());
        Logger.recordOutput(
            "FieldSimulation/Fuel",
            SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel")
        );
    }

    private static boolean resetField() {
        return org.ironmaple.simulation.SimulatedArena.resetFieldPublisher
            .getTopic()
            .getEntry(false)
            .get();
    }
}