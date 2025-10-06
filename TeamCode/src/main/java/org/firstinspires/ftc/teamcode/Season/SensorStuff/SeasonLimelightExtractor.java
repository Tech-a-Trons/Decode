package org.firstinspires.ftc.teamcode.Season.SensorStuff;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.json.JSONArray;
import org.json.JSONObject;

import java.io.InputStream;
import java.io.OutputStream;
import java.net.Socket;
import java.nio.charset.StandardCharsets;
import java.util.concurrent.TimeUnit;

/**
 * Robust Limelight 3A extractor for FTC Control Hub (USB)
 * Fully merges connection, fallback, and reading logic.
 * Automatically clears stale values if no data is received.
 */
public class SeasonLimelightExtractor {

    // Fallback hosts
    private final String[] FALLBACK_HOSTS = {
            "limelight.local",   // USB/mDNS
            "10.9.30.2",         // example DHCP/static fallback
            "10.9.30.11",         // common FTC convention
            "172.29.0.1",
            "172.28.0.1",
            "172.29.0.22"
    };

    private final int PORT = 5810;
    private final int RECONNECT_DELAY_MS = 2000;
    private final double SMOOTHING_FACTOR = 0.2;
    private final long STALE_TIMEOUT_MS = 500; // 0.5s without messages = stale

    // Current host
    private String HOST = FALLBACK_HOSTS[0];
    private Telemetry opModeTelemetry;

    // Network
    private volatile Socket socket;
    private volatile InputStream in;
    private volatile OutputStream out;
    private volatile boolean targetVisible = false;
    private volatile double horizontalAngle = 0.0;
    private volatile double verticalAngle = 0.0;
    private volatile double distanceEstimate = 0.0; // can be computed if target height known

    // Limelight values
    private volatile Double tx = null;
    private volatile Double ty = null;
    private volatile Double ta = null;
    private volatile Double tl = null;
    private Double fps = null;

    // Status
    private volatile String connectionStatus = "Not connected";
    private volatile long lastUpdateTime = 0;

    // Background thread
    private Thread readingThread;
    private volatile boolean running = false;

    // ------------------- PUBLIC METHODS -------------------

    /**
     * Starts reading Limelight values in a background thread.
     * Automatically handles connection, host fallback, NT4 handshake, reading, and stale-value reset.
     */

    public void setTelemetry(Telemetry telemetry) {
        this.opModeTelemetry = telemetry;
    }

    public void startReading() {
        if (running) return;
        running = true;

        readingThread = new Thread(() -> {
            int hostIndex = 0;

            while (running && !Thread.currentThread().isInterrupted()) {
                try {
                    // Connect if not connected
                    if (socket == null || socket.isClosed()) {
                        HOST = FALLBACK_HOSTS[hostIndex];
                        hostIndex = (hostIndex + 1) % FALLBACK_HOSTS.length;

                        try {
                            connectionStatus = "Connecting to " + HOST + "...";
                            socket = new Socket(HOST, PORT);
                            in = socket.getInputStream();
                            out = socket.getOutputStream();

                            // NT4 handshake
                            String hello = "{\"op\":\"hello\",\"protocol\":\"NT4\",\"version\":0}\n";
                            out.write(hello.getBytes(StandardCharsets.UTF_8));
                            out.flush();

                            // Subscribe to topics
                            subscribe("tx");
                            subscribe("ty");
                            subscribe("ta");
                            subscribe("tl");

                            connectionStatus = "Connected to " + HOST;

                        } catch (Exception e) {
                            connectionStatus = "Failed to connect to " + HOST;
                            close();
                            TimeUnit.MILLISECONDS.sleep(RECONNECT_DELAY_MS);
                            continue; // try next host
                        }
                    }

                    // Read messages if connected
                    if (socket != null && !socket.isClosed()) {
                        readMessages();
                    }

                    // Clear stale values if no data recently
                    if (System.currentTimeMillis() - lastUpdateTime > STALE_TIMEOUT_MS) {
                        tx = null;
                        ty = null;
                        ta = null;
                        tl = null;
                        fps = null;
                        connectionStatus = "No data (stale)";
                    }

                } catch (Exception ignored) {}

                try {
                    Thread.sleep(10); // prevent CPU hog
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                }
            }
        });

        readingThread.start();
    }

    /**
     * Stops the background reading thread and closes the socket.
     */
    public void stopReading() {
        running = false;
        if (readingThread != null) {
            readingThread.interrupt();
            readingThread = null;
        }
        close();
    }

    /*
     * Updates internal state for TeleOp.
     * Call this inside your loop() method every iteration.
     * Handles telemetry, stale-value check, and internal updates.
     */
    // Optional derived values for TeleOp

    /*
     * Updates internal state for TeleOp.
     * Call this inside loop() every iteration.
     * Handles telemetry, stale-value reset, and calculates derived values.
     */
    /**
     * Updates internal state for TeleOp.
     * Handles telemetry, stale-value reset, and calculates smoothed derived values.
     */
    /**
     * Updates internal state for TeleOp and automatically refreshes telemetry.
     * Call this every loop; no need to call telemetry.update() separately.
     */
    public void update() {
        long now = System.currentTimeMillis();

        // 1. Clear stale values if no data received recently
        if (now - lastUpdateTime > STALE_TIMEOUT_MS) {
            tx = null;
            ty = null;
            ta = null;
            tl = null;
            fps = null;
            connectionStatus = "No data (stale)";
            targetVisible = false;
            horizontalAngle = 0.0;
            verticalAngle = 0.0;
            distanceEstimate = 0.0;
        } else {
            // 2. Update target visibility
            targetVisible = (ta != null && ta > 0.0);

            // 3. Compute raw derived values
            double rawHorizontal = tx != null ? tx : 0.0;
            double rawVertical = ty != null ? ty : 0.0;

            // Distance estimate
            final double TARGET_HEIGHT = 24.0; // inches (example)
            final double CAMERA_HEIGHT = 9.0;  // inches (example)
            final double CAMERA_ANGLE = 20.0;  // degrees mounting angle
            double rawDistance = 0.0;

            if (targetVisible) {
                double angleToTarget = CAMERA_ANGLE + rawVertical;
                rawDistance = (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(Math.toRadians(angleToTarget));
            }

            // 4. Apply smoothing
            horizontalAngle = smooth(horizontalAngle, rawHorizontal);
            verticalAngle = smooth(verticalAngle, rawVertical);
            distanceEstimate = smooth(distanceEstimate, rawDistance);
        }

        // 5. Refresh telemetry
        if (opModeTelemetry != null) {
            addTelemetry(opModeTelemetry); // optionally pass telemetry into addTelemetry()
            opModeTelemetry.addData("Target Visible", targetVisible);
            opModeTelemetry.addData("Horizontal Angle", String.format("%.2f", horizontalAngle));
            opModeTelemetry.addData("Vertical Angle", String.format("%.2f", verticalAngle));
            opModeTelemetry.addData("Distance Estimate", String.format("%.2f", distanceEstimate));
            opModeTelemetry.update();
        }
    }

    // ------------------- PRIVATE METHODS -------------------

    private void subscribe(String topic) throws Exception {
        String msg = String.format("{\"op\":\"subscribe\",\"topics\":[\"/%s\"]}\n", topic);
        out.write(msg.getBytes(StandardCharsets.UTF_8));
        out.flush();
    }

    private void readMessages() {
        try {
            byte[] buffer = new byte[2048];
            if (in.available() <= 0) return;

            int len = in.read(buffer);
            if (len <= 0) return;

            String raw = new String(buffer, 0, len, StandardCharsets.UTF_8);
            String[] messages = raw.split("\n");

            for (String msg : messages) {
                parseMessage(msg);
            }

        } catch (Exception e) {
            connectionStatus = "Error reading: " + e.getMessage();
            close();
        }
    }

    private void parseMessage(String msg) {
        try {
            JSONObject json = new JSONObject(msg);
            if (!json.has("topic") || !json.has("value")) return;

            String topic = json.getString("topic");
            JSONArray values = json.getJSONArray("value");
            double val = values.getDouble(0);

            switch (topic) {
                case "tx": tx = smooth(tx, val); break;
                case "ty": ty = smooth(ty, val); break;
                case "ta": ta = smooth(ta, val); break;
                case "tl":
                    tl = smooth(tl, val);
                    if (tl > 0) {
                        double newFps = 1000.0 / tl;
                        fps = fps == null ? newFps : fps * 0.8 + newFps * 0.2;
                    }
                    break;
            }

            lastUpdateTime = System.currentTimeMillis();
            connectionStatus = "Receiving data";

        } catch (Exception ignored) {}
    }

    /**
     * Adds current Limelight data to telemetry.
     */
    private void addTelemetry(Telemetry telemetry) {
        if (telemetry == null) return;
        telemetry.addData("Limelight Status", connectionStatus != null ? connectionStatus : "N/A");
        telemetry.addData("tx", tx != null ? String.format("%.2f", tx) : "N/A");
        telemetry.addData("ty", ty != null ? String.format("%.2f", ty) : "N/A");
        telemetry.addData("ta", ta != null ? String.format("%.2f", ta) : "N/A");
        telemetry.addData("tl", tl != null ? String.format("%.2f", tl) : "N/A");
        telemetry.addData("FPS", fps != null ? String.format("%.1f", fps) : "N/A");
    }


    private double smooth(Double oldVal, double newVal) {
        if (oldVal == null) return newVal;
        return oldVal * (1.0 - SMOOTHING_FACTOR) + newVal * SMOOTHING_FACTOR;
    }

    private void close() {
        try { if (socket != null) socket.close(); } catch (Exception ignored) {}
        socket = null; in = null; out = null;
        connectionStatus = "Closed";
    }

    // ------------------- GETTERS -------------------
    public Double getTx() { return tx; }
    public Double getTy() { return ty; }
    public Double getTa() { return ta; }
    public Double getTl() { return tl; }
    public Double getFps() { return fps; }
    public String getStatus() { return connectionStatus; }
}