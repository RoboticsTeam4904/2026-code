package org.usfirst.frc4904.standard.humaninput;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.CommandRobotBase;
import org.usfirst.frc4904.standard.custom.Nameable;

/**
 * A generic human interface class. This is designed to be used to bind commands
 * to controllers. bindCommands should only be called during teleop init.
 */
public abstract class HumanInput implements Nameable {

    protected final String name;

    public HumanInput(String name) {
        this.name = name;
    }

    @Override
    public String getName() {
        return name;
    }

    /**
     * A function where the driver's and operator's controls are bound to commands
     * <p>
     * Can't be done in the constructor because constructors are called too early
     */
    public abstract void bindCommands();

    /**
     * Can be optionally overridden to unbind commands that were bound in bindCommands().
     * <p>
     * You should do this for any commands that were <em>not</em> bound to one of the following:
     * <ul>
     *   <li> {@link RobotMap.HumanInput.Driver#xyJoystick}
     *   <li> {@link RobotMap.HumanInput.Driver#turnJoystick}
     *   <li> {@link RobotMap.HumanInput.Operator#joystick}
     * </ul>
     * See {@link CommandRobotBase#clearBindings()}
     */
    public void unbindCommands() {}
}
