package org.usfirst.frc4904.standard.custom.controllers;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A joystick that implements the generic controller interface and the 2023
 * trigger interface. This allows us to use a joystick as a controller. This
 * contains 12 buttons to reflect the joysticks we are typically using.
 */
public class CustomCommandPS4 extends CommandPS4Controller {

    // public so that the loop can be used with custom triggers. for example:
    // new Trigger(joystick.loop, () -> joystick.getAxis(Axis.X) > 0.5)
    public final EventLoop loop;

    protected final double deadzone;

    public CustomCommandPS4(int port, double deadzone) {
        super(port);
        if (deadzone < 0 || deadzone > 1) {
            throw new IllegalArgumentException("Joystick deadzone must be in [0, 1]");
        }
        this.deadzone = deadzone;

        loop = new EventLoop();
        CommandScheduler.getInstance().getDefaultButtonLoop().bind(loop::poll);
    }

    @Override
    public Trigger button(int id) {
        return super.button(id, loop);
    }

    @Override
    public Trigger button(int button, EventLoop loop) {
        // clearBindings() won't work if the button is on a different event loop
        throw new UnsupportedOperationException("CustomCommandJoystick.button() does not support setting a custom event loop.");
    }

    public void clearBindings() {
        loop.clear();
    }

    /**
     * Get the X axis value of right side of the controller.
     *
     * @return The axis value.
     */
    @Override
    public double getRightX() {
        return deadzone(getRightX(), deadzone);
    }

    /**
     * Get the Y axis value of left side of the controller.
     *
     * @return The axis value.
     */
    @Override
    public double getLeftY() {
        return deadzone(getLeftY(), deadzone);
    }

    /**
     * Get the Y axis value of right side of the controller.
     *
     * @return The axis value.
     */
    @Override
    public double getRightY() {
        return deadzone(getRightY(), deadzone);
    }

    public static double deadzone(double input, double deadzone) {
        if (Math.abs(input) < deadzone) return 0;
        return (input - Math.signum(input) * deadzone) / (1 - deadzone); // linear between 0 and 1 in the remaining range
    }
}
