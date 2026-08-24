package robot.humaninterface.drivers;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import lib.commands.AlwaysRunnableInstantCommand;
import lib.commands.conditional.RunIf;
import lib.commands.conditional.RunUnless;
import lib.custom.controllers.CustomCommandPS4;
import lib.humaninput.Driver;
import robot.RobotMap.Component;
import robot.RobotMap.HumanInput;
import robot.humaninterface.operators.DefaultOperator;
import robot.subsystems.ShooterSubsystem;

public class PS4Driver extends Driver {

    private static final double SPEED_EXP = 2, TURN_EXP = 2;

    private static final double DRIVE_DEADZONE = 0.02, TURN_DEADZONE = 0.02;

    @Override
    public void bindCommands() {
        CustomCommandPS4 ps4 = HumanInput.Driver.ps4;

        Component.chassis.setDefaultCommand(
            Component.chassis.c_input(this::getTranslation, this::getTurnSpeed)
                .withName("Driver - swerve drive")
        );

        ps4.povDown().whileTrue(new StartEndCommand(
            () -> Component.chassis.setAutoBrickWhenStill(true),
            () -> Component.chassis.setAutoBrickWhenStill(false)
        ));

        // navx reset
        ps4.povUp().onTrue(
            new AlwaysRunnableInstantCommand(() -> Component.chassis.resetOdometry())
        );

        // flip zeroes
        ps4.povLeft().onTrue(
            new RunUnless(
                new AlwaysRunnableInstantCommand(() -> Component.chassis.flipZero()),
                DriverStation::isTeleopEnabled
            )
        );

        // swerve reset
        ps4.povRight().onTrue(
            new RunUnless(
                new AlwaysRunnableInstantCommand(() -> Component.chassis.zero()),
                DriverStation::isTeleopEnabled
            )
        );

        // ps4.povRight().onTrue(
        //     new AlwaysRunnableInstantCommand(() -> edu.wpi.first.wpilibj.Preferences.setDouble("cheese", 5))
        // );
        // ps4.povRight().whileTrue(
        //     Component.INTAKE_DO_NOT_USE_OR_YOU_WILL_BE_FIRED.c_forward(true)
        // );

        // climber up
        ps4.triangle().onTrue(Component.climber.c_gotoUp());
        // climber down
        ps4.cross().onTrue(Component.climber.c_gotoDown());

        // long shoot
        ps4.circle().whileTrue(DefaultOperator.wrapShootCommand(Component.shooter.c_longShoot()));

        // indexer
        ps4.square().whileTrue(Component.indexer.c_forward(true));

        // intake retract
        ps4.L1().onTrue(Component.intake.c_retract());
        // intake extend
        ps4.L2().onTrue(Component.intake.c_extend());
        // run intake while any of the above are held
        ps4.L1().or(ps4.L2()).whileTrue(Component.intake.c_intake());

        // align
        ps4.R1().whileTrue(ShooterSubsystem.c_smartShootAlign());

        // index and shooter
        ps4.R2().whileTrue(DefaultOperator.c_smartShoot());
        ps4.R2().whileFalse(
            new RunIf(
                Component.shooter.c_smartShoot().withTimeout(0.7),
                Component.shooter::canShoot
            )
        );
    }

    @Override
    public void unbindCommands() {
        Component.chassis.removeDefaultCommand();
    }

    protected double getRawForward() {
        return -HumanInput.Driver.ps4.getRawLeftY();
    }
    protected double getRawLeft() {
        return -HumanInput.Driver.ps4.getRawLeftX();
    }
    protected double getRawTurn() {
        return -HumanInput.Driver.ps4.getRawRightX();
    }

    @Override
    public Translation2d getTranslation() {
        Translation2d translation = new Translation2d(getRawForward(), getRawLeft());
        double mag = translation.getNorm();
        if (mag == 0) return translation;

        double len = scaleGain(mag, SPEED_EXP, DRIVE_DEADZONE);
        return translation.times(len / mag); // unit translation * len
    }

    @Override
    public double getTurnSpeed() {
        return scaleGain(getRawTurn(), TURN_EXP, TURN_DEADZONE);
    }
}
