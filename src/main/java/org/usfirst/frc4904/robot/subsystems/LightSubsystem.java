package org.usfirst.frc4904.robot.subsystems;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.usfirst.frc4904.standard.Perlin2D;
import org.usfirst.frc4904.standard.Util;

public class LightSubsystem extends SubsystemBase {

    public static class Color {

        public static final int[] SUCCESS  = { 0, 200, 50 };
        public static final int[] FAIL     = { 255, 50, 0 };

        public static final int[] VISION   = { 127, 0, 255 };

        public static final int[] ENABLED  = { 255, 100, 0 };
        public static final int[] DISABLED = { 200, 220, 255 };

    }

    public double visionProgress = -1;

    private float[] flashColor = new float[4];
    private float flashStrength = 0;

    double lastUpdateTime;

    private static class BufferViewData {

        public final float[][] colorArray;
        public final AddressableLEDBufferView view;

        public BufferViewData(AddressableLEDBuffer buffer, int start, int length, boolean reverse) {
            colorArray = new float[length][4];

            int end = start + length - 1;
            view = buffer.createView(reverse ? end : start, reverse ? start : end);
        }

        public void copyColorsToBuffer() {
            for (int i = 0; i < colorArray.length; i++) {
                float[] color = colorArray[i];
                float a = color[3];

                view.setRGB(
                    i,
                    (int) (Math.pow(color[0] * a, 2) * 255),
                    (int) (Math.pow(color[1] * a, 2) * 255),
                    (int) (Math.pow(color[2] * a, 2) * 255)
                );
            }
        }

    }

    final AddressableLED led;
    final AddressableLEDBuffer buffer;
    final BufferViewData[] views;

    public LightSubsystem(AddressableLED led, int ledLength, int[] lengths, boolean[] reverse) {
        this.led = led;
        buffer = new AddressableLEDBuffer(ledLength);

        led.setLength(ledLength);
        led.start();

        this.views = new BufferViewData[lengths.length];
        int start = 0;
        for (int i = 0; i < lengths.length; i++) {
            this.views[i] = new BufferViewData(buffer, start, lengths[i], reverse[i]);
            start += lengths[i];
        }

        lastUpdateTime = Timer.getFPGATimestamp();
    }

    private void alphaBlend(float[][] colors, float[] c2, float mix) {
        float a2 = c2[3] * mix;

        for (float[] c1 : colors) {
            float a1 = c1[3];
            float a = a2 + a1 * (1 - a2);

            c1[0] = (c2[0] * a2 + c1[0] * a1 * (1 - a2)) / a;
            c1[1] = (c2[1] * a2 + c1[1] * a1 * (1 - a2)) / a;
            c1[2] = (c2[2] * a2 + c1[2] * a1 * (1 - a2)) / a;
            c1[3] = a;
        }
    }

    private void progressBar(float[][] colors, float progress, int[] color) {
        int length = colors.length;

        for (int i = 0; i < length; i++) {
            float strength = Util.clamp(progress * length - i, 0, 1);
            if (strength > 0) {
                colors[i][0] = color[0] / 255f;
                colors[i][1] = color[1] / 255f;
                colors[i][2] = color[2] / 255f;
            }
            float alpha = color.length == 4 ? color[3] : 1;
            colors[i][3] = alpha * strength;
        }
    }

    private final Perlin2D fireNoise = new Perlin2D(12345678987654321L);

    private void fire(float[][] colors, boolean blue) {
        float time = (float) lastUpdateTime;

        for (int i = 0; i < colors.length; i++) {
            float height = 1 - (float) i / (colors.length - 1);
            float noise = fireNoise.noise(time, -i * 0.13f + time * 2);
            float strength = (float) Math.pow(noise, 2.5) * 1.3f + height - 0.55f;

            float r = 1;
            float g = Util.clamp(strength * 2 - 0.5f, 0, 1);
            float b = Util.clamp(strength * 4 - 3, 0, 1);
            float a = Util.clamp(strength * 4, 0, 1);


            //TODO: make "blue" and "!blue depending on alliance colour"
            colors[i][0] = blue ? b : r;
            colors[i][1] = g * 0.8f; // green LEDs are brighter
            colors[i][2] = blue ? r : b;
            colors[i][3] = a;
        }
    }

    /**
     * Flash an RGB color for about a second. Color is an array of 3 (RGB) or 4 (RGBA) ints from 0-255.
     */
    public void flashColor(int[] color) {
        if (color.length == 3) {
            flashColor(color[0], color[1], color[2]);
        } else if (color.length == 4) {
            flashColor(color[0], color[1], color[2], color[3]);
        } else {
            System.err.println("LightSubsystem.flashColor(int[] color) must take an array of 3 or 4 ints for RGB or RGBA");
            flashColor(0, 0, 0);
        }
    }

    /**
     * Flash an RGB color for about a second. Colors are from 0-255.
     */
    public void flashColor(int r, int g, int b) {
        flashColor(r, g, b, 255);
    }

    /**
     * Flash an RGB color for about a second. Colors and alpha are from 0-255.
     */
    public void flashColor(int r, int g, int b, int a) {
        flashColor = new float[] { r / 255f, g / 255f, b / 255f, a / 255f };
        flashStrength = 1;
    }

    private final LEDPattern rainbowPattern =
        LEDPattern.rainbow(255, 128).scrollAtAbsoluteSpeed(Units.MetersPerSecond.of(0.3), Units.Meters.of(1.0 / 60));

    @Override
    public void periodic() {
        double time = Timer.getFPGATimestamp();
        double deltaTime = time - lastUpdateTime;
        lastUpdateTime = time;

        if (DriverStation.isDisabled()) {
            rainbowPattern.applyTo(buffer);
        } else {
            for (var view : views) {
                float[][] colors = view.colorArray;

                // if (visionProgress != -1) {
                //     progressBar(colors, (float) visionProgress, Color.VISION);
                // } else {
                fire(colors, DriverStation.isAutonomous());
                // }

                if (flashStrength > 0) {
                    alphaBlend(colors, flashColor, (float) Math.sqrt(flashStrength));
                    flashStrength -= (float) deltaTime / 1.5;
                }

                view.copyColorsToBuffer();
            }
        }

        led.setData(buffer);
    }

}
