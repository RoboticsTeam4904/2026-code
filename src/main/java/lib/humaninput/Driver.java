package lib.humaninput;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Driver specific version of HumanInput
 */
public abstract class Driver extends HumanInput {

    public Driver() {
        super();
    }

    public Driver(String name) {
        super(name);
    }

    /**
     * @return Speed that the driver wants, in WPILib field-relative coordinates (forward, left).
     *         Length of translation vector should not exceed 1
     */
    public abstract Translation2d getTranslation();

    /**
     * @return Turn speed that the driver wants in the range [-1, 1], positive = counterclockwise
     */
    public abstract double getTurnSpeed();

}
