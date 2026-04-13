package lib.custom;

/**
 * Class for interfacing with a Teensy running TeensyNeoPixelCAN code.
 */
public abstract class CustomNeoPixels extends CustomCAN {

    protected byte R = 0;
    protected byte G = 0;
    protected byte B = 0;
    protected int mode = 0;
    protected int value = 0;

    /**
     * Constructor ID should be between 0x600 and 0x700.
     *
     * @param name
     * @param id
     */
    public CustomNeoPixels(String name, int id) {
        super(name, id);
    }

    protected final CustomNeoPixels setMode(int mode) { // People should be forced to overwrite this with a more user friendly mode
        // system
        this.mode = mode;

        return this;
    }

    /**
     * Sets the color of the pattern. Values are 0-255.
     *
     * @param R
     * @param G
     * @param B
     */
    public CustomNeoPixels setColor(int R, int G, int B) {
        this.R = (byte) R;
        this.G = (byte) G;
        this.B = (byte) B;

        return this;
    }

    /**
     * Sets the progress of the pattern. Values are 0 to 32768.
     *
     * @param value
     */
    public CustomNeoPixels setValue(int value) {
        this.value = value;

        return this;
    }

    /**
     * Writes LED pattern to Teensy.
     */
    public void update() {
        super.write(
            new byte[] {
                B,
                G,
                R,
                0x00,
                (byte) (value >> 8),
                (byte) (value & 0xFF),
                (byte) (mode >> 8),
                (byte) (mode & 0xFF),
            }
        );
    }
}
