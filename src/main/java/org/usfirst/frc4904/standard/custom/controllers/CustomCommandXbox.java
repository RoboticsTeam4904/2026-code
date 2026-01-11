package org.usfirst.frc4904.standard.custom.controllers;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class CustomCommandXbox extends CommandXboxController {
    private final double deadzone;
    private final CommandXboxController hid;

    public CustomCommandXbox(int port, double deadzone) {
        super(port);
        hid = new CommandXboxController(port);
        if (deadzone < 0 || deadzone >= 1) {
            throw new IllegalArgumentException("CustomCommandXbox deadzone must be in [0, 1]");
        }
        this.deadzone = deadzone;
    }

    public double getLeftX() {
        return applyDeadzone(hid.getLeftX(), deadzone);
    }

    /**
     * Get the X axis value of right side of the controller.
     *
     * @return The axis value.
     */
    public double getRightX() {
        return applyDeadzone(hid.getRightX(), deadzone);
    }

    /**
     * Get the Y axis value of left side of the controller.
     *
     * @return The axis value.
     */
    public double getLeftY() {
        return applyDeadzone(hid.getLeftY(), deadzone);
    }

    /**
     * Get the Y axis value of right side of the controller.
     *
     * @return The axis value.
     */
    public double getRightY() {
        return applyDeadzone(hid.getRightY(), deadzone);
    }

    @Override
    public double getRightTriggerAxis() {
        return applyDeadzone(hid.getRightTriggerAxis(), deadzone);
    }

    @Override
    public double getLeftTriggerAxis() {
        return applyDeadzone(hid.getLeftTriggerAxis(), deadzone);
    }

    public static double applyDeadzone(double input, double deadzone) {
        if (Math.abs(input) < deadzone) return 0;
        return (input - Math.signum(input) * deadzone) / (1 - deadzone); // linear between 0 and 1 in the remaining range
    }
}
