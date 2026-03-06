package org.usfirst.frc4904.robot.Simulation;

import org.usfirst.frc4904.robot.subsystems.FolderIO.ClimberTwo;
import org.usfirst.frc4904.robot.subsystems.IO.ArmState;
import org.usfirst.frc4904.robot.subsystems.IO.ArmSubsystem;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class MechanismSimulator {
    private final Mechanism2d panel;
    private final MechanismRoot2d armroot;
    private final MechanismLigament2d arm;
    private final MechanismRoot2d climbroot;
    private final MechanismLigament2d climber;

    private final ArmSubsystem armSubsystem;

    public MechanismSimulator(ArmSubsystem armS, ClimberTwo climbS) {
        this.armSubsystem = armS;
        this.ClimberTwo = climbS;

        this.panel = new Mechanism2d(
            Units.inchesToMeters(100), 
            Units.inchesToMeters(100)
        );
       
        this.armroot = panel.getRoot(
            "arm", 
            Units.inchesToMeters(7.35), 
            Units.inchesToMeters(10)
        );

        this.climbroot = panel.getRoot(
            "climber",
             Units.inchesToMeters(7.35),
             Units.inchesToMeters(10) );

        this.arm = armroot.append(
            new MechanismLigament2d("arm", Units.inchesToMeters(25), 0, 6, new Color8Bit(Color.kYellow))
        );

        this.climber = climbroot.append(
            new MechanismLigament2d("climber", 
            Units.inchesToMeters(25), 0, 3, new Color8Bit(Color.kLightGoldenrodYellow))
        );
    }

    public void periodic() {
        ArmState.InputState currentArmState = armSubsystem.getState();
        if (currentArmState != null) {
            this.arm.setAngle(currentArmState.currentAngleDegrees());
        }

        ClimbState.InputState currentClimberState = climberTwo.getState();
        if (currentClimberState != null) {
            this.climber.setAngle(currentClimberState.currentHeight());
        }

        SmartDashboard.putData("Robot Simulation", panel);
    }
}
