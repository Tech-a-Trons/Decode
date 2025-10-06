package org.firstinspires.ftc.teamcode.Season;

import org.json.JSONObject;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.Socket;
import java.nio.charset.StandardCharsets;
import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * Limelight extractor for FTC (no NetworkTables dependency)
 * Supports auto-detecting topic prefixes and rich debugging.
 */
public class SeasonLimelightExtractor {
    private static final String HOST = "172.29.0.1"; // <- Replace with your Limelight’s IP
    private static final int PORT = 5810;
    private static final int RECONNECT_DELAY_MS = 2000;

    private volatile boolean running = false;
    private Thread listenerThread;
    private volatile Socket socket;
    private volatile InputStream in;
    private volatile OutputStream out;

    private volatile Double tx = null, ty = null, ta = null;
    private volatile Double tl = null;   // latency in ms
    private volatile Double fps = null;  // calculated frames per second

    private volatile String connectionStatus = "Not started";

    public void start() {
        if (running) return;
        running = true;
        listenerThread = new Thread(this::runLoop, "LimelightListener");
        listenerThread.start();
        log("Listener thread started.");
    }

    private void runLoop() {
        while (running) {
            try {
                connect();
                listenLoop();
            } catch (Exception e) {
                log("⚠️ Connection error: " + e.getMessage());
                closeSocket();
                connectionStatus = "Disconnected, retrying...";
                try { Thread.sleep(RECONNECT_DELAY_MS); } catch (InterruptedException ignored) {}
            }
        }
    }

    private void connect() throws Exception {
        log("Connecting to " + HOST + ":" + PORT + " ...");
        socket = new Socket(HOST, PORT);
        in = socket.getInputStream();
        out = socket.getOutputStream();

        // NT4 handshake
        String hello = "{\"op\":\"hello\",\"protocol\":\"NT4\",\"version\":0}\n";
        out.write(hello.getBytes(StandardCharsets.UTF_8));
        out.flush();

        // Subscribe broadly — not all Limelights use same prefix
        subscribe("limelight/tx");
        subscribe("limelight/ty");
        subscribe("limelight/ta");
        subscribe("tx");
        subscribe("ty");
        subscribe("ta");
        subscribe("limelight/tl");
        subscribe("tl");


        log("✅ Connected and subscribed.");
        connectionStatus = "Connected";
    }

    private void subscribe(String topic) throws Exception {
        String msg = String.format("{\"op\":\"subscribe\",\"topics\":[\"/%s\"]}\n", topic);
        out.write(msg.getBytes(StandardCharsets.UTF_8));
        out.flush();
    }

    private void listenLoop() throws Exception {
        byte[] buffer = new byte[4096];
        while (running && socket != null && !socket.isClosed()) {
            int len = in.read(buffer);
            if (len <= 0) throw new Exception("Socket closed or no data");

            String msg = new String(buffer, 0, len, StandardCharsets.UTF_8);
            parseMessage(msg);
        }
    }

    private void parseMessage(String msg) {
        try {
            JSONObject json = new JSONObject(msg);
            if (!json.has("topic")) {return;}

            String topic = json.getString("topic");
            if (!json.has("value")) {return;}

            double value = json.getJSONArray("value").getDouble(0);

            if (topic.contains("tx")) {
                tx = value;
            } else if (topic.contains("ty")) {
                ty = value;
            } else if (topic.contains("ta")) {
                ta = value;
            } else if (topic.contains("tl")) {
                tl = value;
                if (tl != null && tl > 0) {
                    double newFps = 1000.0 / tl;
                    if (fps == null) fps = newFps;
                    else fps = fps * 0.8 + newFps * 0.2; // smooth moving average
                }
            } else {log("🔍 Unrecognized topic: " + topic);}

            if (tx != null || ty != null || ta != null || tl != null) {
                connectionStatus = "Receiving data";
            }
        } catch (Exception e) {
            log("Parse error: " + e.getMessage());
        }
    }

    public Double getTx() { return tx; }
    public Double getTy() { return ty; }
    public Double getTa() { return ta; }
    public Double getTl() { return tl; }
    public Double getFps() { return fps; }
    public String getStatus() { return connectionStatus; }

    public void stop() {
        running = false;
        closeSocket();
        try { if (listenerThread != null) {listenerThread.join(200);} } catch (InterruptedException ignored) {}
        log("Stopped.");
    }

    private void closeSocket() {
        try { if (socket != null) {socket.close();} } catch (Exception ignored) {}
        socket = null; in = null; out = null;
    }

    private void log(String msg) {
        String time = new SimpleDateFormat("HH:mm:ss").format(new Date());
        System.out.println("[Limelight][" + time + "] " + msg);
    }
}