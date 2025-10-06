package org.firstinspires.ftc.teamcode.Season;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.json.JSONArray;
import org.json.JSONObject;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import java.io.InputStream;
import java.io.OutputStream;
import java.net.Socket;
import java.nio.charset.StandardCharsets;

/**
 * Robust Limelight 3A extractor for Control Hub (USB)
 * Works over NT4 virtual network (port 5810)
 */
public class SeasonLimelightExtractor {

    private static final String HOST = "179.29.0.22"; // Replace with your Limelight IP
    private static final int PORT = 5810;
    private static final int RECONNECT_DELAY_MS = 2000;

    private volatile Socket socket;
    private volatile InputStream in;
    private volatile OutputStream out;

    private volatile Double tx = null;
    private volatile Double ty = null;
    private volatile Double ta = null;
    private volatile Double tl = null;
    private Double fps = null;
    private long lastTime = System.currentTimeMillis();


    private volatile String connectionStatus = "Not connected";

    // For smoothing
    private final double SMOOTHING_FACTOR = 0.2;

    public void update() {
        if (socket == null || socket.isClosed()) {
            tryConnect();
        }
        if (socket != null && !socket.isClosed()) {
            readMessages();
        }
    }

    private void tryConnect() {
        try {
            connectionStatus = "Connecting...";
            socket = new Socket(HOST, PORT);
            in = socket.getInputStream();
            out = socket.getOutputStream();

            // NT4 handshake
            String hello = "{\"op\":\"hello\",\"protocol\":\"NT4\",\"version\":0}\n";
            out.write(hello.getBytes(StandardCharsets.UTF_8));
            out.flush();

            // Subscribe only to required topics
            subscribe("tx");
            subscribe("ty");
            subscribe("ta");
            subscribe("tl");

            connectionStatus = "Connected";
        } catch (Exception e) {
            connectionStatus = "Failed to connect, retrying...";
            close();
            try {
                Thread.sleep(RECONNECT_DELAY_MS);
            } catch (InterruptedException ignored) {}
        }
    }

    private void subscribe(String topic) throws Exception {
        String msg = String.format("{\"op\":\"subscribe\",\"topics\":[\"/%s\"]}\n", topic);
        out.write(msg.getBytes(StandardCharsets.UTF_8));
        out.flush();
    }

    private void readMessages() {
        try {
            byte[] buffer = new byte[2048];
            if (in.available() <= 0) {return;}

            int len = in.read(buffer);
            if (len <= 0) {return;}

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
            if (!json.has("topic") || !json.has("value")) {return;}

            String topic = json.getString("topic");
            JSONArray values = json.getJSONArray("value");
            double val = values.getDouble(0);

            switch (topic) {
                case "tx": tx = smooth(tx, val); break;
                case "ty": ty = smooth(ty, val); break;
                case "ta": ta = smooth(ta, val); break;
                case "tl":
                    tl = smooth(tl, val);

                    // Calculate FPS from latency
                    if (tl > 0) {
                        double newFps = 1000.0 / tl;
                        fps = fps == null ? newFps : fps * 0.8 + newFps * 0.2; // smoothing
                    }
                    break;

            }

            connectionStatus = "Receiving data";

        } catch (Exception ignored) {}
    }

    private double smooth(Double oldVal, double newVal) {
        if (oldVal == null) {return newVal;}
        return oldVal * (1.0 - SMOOTHING_FACTOR) + newVal * SMOOTHING_FACTOR;
    }

    public Double getTx() { return tx; }
    public Double getTy() { return ty; }
    public Double getTa() { return ta; }
    public Double getTl() { return tl; }
    public Double getFps() { return fps; }
    public String getStatus() { return connectionStatus; }

    public void addTelemetry() {
        if (telemetry == null) {return;}  // safety check

        telemetry.addData("Limelight Status", connectionStatus != null ? connectionStatus : "N/A");
        telemetry.addData("tx", tx != null ? String.format("%.2f", tx) : "N/A");
        telemetry.addData("ty", ty != null ? String.format("%.2f", ty) : "N/A");
        telemetry.addData("ta", ta != null ? String.format("%.2f", ta) : "N/A");
        telemetry.addData("tl", tl != null ? String.format("%.2f", tl) : "N/A");
        telemetry.addData("FPS", fps != null ? String.format("%.1f", fps) : "N/A");
    }

    public void close() {
        try { if (socket != null) socket.close(); } catch (Exception ignored) {}
        socket = null; in = null; out = null;
        connectionStatus = "Closed";
    }
}
