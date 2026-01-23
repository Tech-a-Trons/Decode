package org.firstinspires.ftc.teamcode.Season.Prototypes.PreRegionals;

import android.graphics.Color;
import com.qualcomm.hardware.rev.RevColorSensorV3;


/**
 * A small Runnable wrapper that continuously reads the REV Color Sensor V3
 * on its own thread and exposes thread-safe getters.
 *
 * Uses volatile fields for simple, fast concurrency safe reads from the OpMode loop.
 */
public class ColorSensorTask implements Runnable {
    private final RevColorSensorV3 sensor;
    private final Thread thread;
    private volatile boolean running = true;

    // latest raw values (volatile so main thread sees updates)
    private volatile int red = 0;
    private volatile int green = 0;
    private volatile int blue = 0;
    private volatile int alpha = 0;

    // HSV array - keep a copy getter (defensive) to avoid returning the internal array object
    private volatile float hue = 0f;
    private volatile float saturation = 0f;
    private volatile float value = 0f;

    // update interval in ms (tweak as needed)
    private final long updateIntervalMs;

    public ColorSensorTask(RevColorSensorV3 sensor, long updateIntervalMs) {
        this.sensor = sensor;
        this.updateIntervalMs = Math.max(1, updateIntervalMs);
        this.thread = new Thread(this, "ColorSensorTask");
        this.thread.setDaemon(true);
        this.thread.start();
    }

    @Override
    public void run() {
        while (running && !Thread.currentThread().isInterrupted()) {
            try {
                // Read raw values from sensor
                int r = sensor.red();
                int g = sensor.green();
                int b = sensor.blue();
                int a = sensor.alpha();

                // compute hsv (Color.RGBToHSV expects 0..255 ints)
                float[] hsv = new float[3];
                Color.RGBToHSV(r, g, b, hsv);

                // publish (atomic-ish due to volatiles; readers will see a consistent snapshot)
                red = r;
                green = g;
                blue = b;
                alpha = a;
                hue = hsv[0];
                saturation = hsv[1];
                value = hsv[2];

                // sleep to control sample rate
                Thread.sleep(updateIntervalMs);
            } catch (InterruptedException e) {
                // Preserve the interrupt status and exit loop
                Thread.currentThread().interrupt();
                break;
            } catch (Exception ex) {
                // Fail-safe: log or ignore to keep thread alive; in your robot, consider telemetry/logging
                // We don't have telemetry here - in OpMode you can read exception state via getters if you extend task.
            }
        }
    }

    public void stop() {
        running = false;
        thread.interrupt();
    }

    // Getters (fast, non-blocking)
    public int getRed() { return red; }
    public int getGreen() { return green; }
    public int getBlue() { return blue; }
    public int getAlpha() { return alpha; }

    // HSV getters
    public float getHue() { return hue; }
    public float getSaturation() { return saturation; }
    public float getValue() { return value; }

    // Optionally convenience methods
    public int getARGB() {
        return Color.argb(alpha, red, green, blue);
    }
}
