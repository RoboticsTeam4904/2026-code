package org.usfirst.frc4904.standard;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.usfirst.frc4904.standard.custom.CommandSendableChooser;
import org.usfirst.frc4904.standard.custom.NamedSendableChooser;
import org.usfirst.frc4904.standard.humaninput.Driver;
import org.usfirst.frc4904.standard.humaninput.Operator;
import org.usfirst.frc4904.standard.util.CmdUtil;

/**
 * IterativeRobot is normally the base class for command based code, but we
 * think certain features will almost always be needed, so we created the
 * CommandRobotBase class. Robot should extend this instead of iterative robot.
 */
public abstract class CommandRobotBase extends TimedRobot {

	private Command autonCommand;
	protected CommandSendableChooser autoChooser;
	protected NamedSendableChooser<Driver> driverChooser;
	protected NamedSendableChooser<Operator> operatorChooser;

	/**
	 * This displays our choosers. The default choosers are for autonomous type,
	 * driver control, sand operator control.
	 */
	protected final void displayChoosers() {
		SmartDashboard.putData("Auton Routine Selector", autoChooser);
		SmartDashboard.putData("Driver control scheme chooser", driverChooser);
		SmartDashboard.putData("Operator control scheme chooser", operatorChooser);
	}

	// HACK FIXME, incredibly cursed and potentially bad
	public static Driver driverConfig = new Driver("uhohhh") {
		@Override
		public double getX() {
			for (int i = 0; i < 100; i++) System.err.println("DRIVER NOT CONFIGED");
			return 0;
		}

		@Override
		public double getY() {
			return 0;
		}

		@Override
		public double getTurnSpeed() {
			return 0;
		}

		@Override
		public void bindCommands() {}
	};

	/**
	 * This initializes the entire robot. It is called by WPILib on robot code
	 * launch. Year-specific code should be written in the initialize function.
	 */
	@Override
	public final void robotInit() {
		// Initialize choosers
		autoChooser = new CommandSendableChooser();
		driverChooser = new NamedSendableChooser<>();
		operatorChooser = new NamedSendableChooser<>();
		// Run user-provided initialize function
		initialize();
		// Display choosers on SmartDashboard
		displayChoosers();
	}

	// The following methods are 'final' to prevent accidentally overriding robot
	// functionality when implementing year specific code. Instead, use the methods
	// linked in the Javadoc for each method below.

	/** Use {@link #teleopInitialize()} instead. */
	@Override
	public final void teleopInit() {
		if (driverChooser.getSelected() != null) {
			// LogKitten.d("Loading driver " + driverChooser.getSelected().getName());
			CommandRobotBase.driverConfig = driverChooser.getSelected();
			driverChooser.getSelected().bindCommands();
		}
		if (operatorChooser.getSelected() != null) {
			// LogKitten.d("Loading operator " + operatorChooser.getSelected().getName());
			operatorChooser.getSelected().bindCommands();
		}
		teleopInitialize();
	}

	/** Use {@link #teleopExecute()} instead. */
	@Override
	public final void teleopPeriodic() {
		CommandScheduler.getInstance().run();
		teleopExecute();
		alwaysExecute();
	}

	/** Use {@link #teleopCleanup()} instead. */
	@Override
	public final void teleopExit() {
		teleopCleanup();
	}

	/** Use {@link #autonomousInitialize()} instead. */
	@Override
	public final void autonomousInit() {
		autonCommand = autoChooser.getSelected();
		if (autonCommand != null) {
			CmdUtil.schedule(autonCommand);
		}

		autonomousInitialize();
	}

	/** Use {@link #autonomousExecute()} instead. */
	@Override
	public final void autonomousPeriodic() {
		CommandScheduler.getInstance().run();
		autonomousExecute();
		alwaysExecute();
	}

	/** Use {@link #autonomousCleanup()} instead. */
	@Override
	public final void autonomousExit() {
		if (autonCommand != null) {
			autonCommand.cancel();
		}

		autonomousCleanup();
	}

	/** Use {@link #disabledInitialize()} instead. */
	@Override
	public final void disabledInit() {
		disabledInitialize();
	}

	/** Use {@link #disabledExecute()} instead. */
	@Override
	public final void disabledPeriodic() {
		CommandScheduler.getInstance().run();
		disabledExecute();
		alwaysExecute();
	}

	/** Use {@link #disabledCleanup()} instead. */
	@Override
	public final void disabledExit() {
		disabledCleanup();
	}

	/** Use {@link #testInitialize()} instead. */
	@Override
	public final void testInit() {
		testInitialize();
	}

	/** Use {@link #testExecute()} instead. */
	@Override
	public final void testPeriodic() {
		CommandScheduler.getInstance().run();
		testExecute();
		alwaysExecute();
	}

	/** Use {@link #testCleanup()} instead. */
	@Override
	public final void testExit() {
		testCleanup();
	}


	/** Function for year-specific code to be run on robot code launch. */
	public abstract void initialize();

	/** Function for year-specific code to be run in every robot mode. */
	public abstract void alwaysExecute();


	/** Function for year-specific code to be run on teleoperated initialize. */
	public abstract void teleopInitialize();

	/** Function for year-specific code to be run during teleoperated time. */
	public abstract void teleopExecute();

	/** Function for year-specific code to be run when teleoperated mode ends. */
	public abstract void teleopCleanup();


	/** Function for year-specific code to be run on autonomous initialize. */
	public abstract void autonomousInitialize();

	/** Function for year-specific code to be run during autonomous. */
	public abstract void autonomousExecute();

	/** Function for year-specific code to be run when autonomous mode ends. */
	public abstract void autonomousCleanup();


	/** Function for year-specific code to be run on disabled initialize. */
	public abstract void disabledInitialize();

	/** Function for year-specific code to be run while disabled. */
	public abstract void disabledExecute();

	/** Function for year-specific code to be run when disabled mode ends. */
	public abstract void disabledCleanup();


	/** Function for year-specific code to be run on disabled initialize. */
	public abstract void testInitialize();

	/** Function for year-specific code to be run while in test mode. */
	public abstract void testExecute();

	/** Function for year-specific code to be run when test mode ends. */
	public abstract void testCleanup();

}
