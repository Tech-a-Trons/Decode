package org.firstinspires.ftc.teamcode.Season;

import java.io.InputStream;
import java.io.OutputStream;
import java.net.Socket;
import java.nio.charset.StandardCharsets;

import java.io.InputStream;
import java.io.OutputStream;
import java.net.Socket;
import java.nio.charset.StandardCharsets;
import org.json.JSONObject;

/**
 * Robust Limelight data extractor for FTC (no NetworkTables library).
 * Continuously retrieves tx, ty, and ta via Limelight's NT4 socket (port 5810).
 */
public class SeasonLimelightExtractor {
    private static final String HOST = "limelight.local"; // or static IP (e.g. "192.168.43.11")
    private static final int PORT = 5810;
    private static final int RECONNECT_DELAY_MS = 2000;    // 2 seconds between reconnect attempts
    private volatile boolean running = false;
    private Thread listenerThread;
    private volatile Socket socket;
    private volatile InputStream in;
    private volatile OutputStream out;
    private volatile Double tx = null;
    private volatile Double ty = null;
    private volatile Double ta = null;

    public void start() {
        running = true;
        listenerThread = new Thread(this::runLoop);
        listenerThread.start();
    }

    public void runLoop() {
        while (running) {
            try {
                connect();
                listenLoop();
            } catch (Exception e) {
                // Connection lost or failed — retry after short delay
                closeSocket();
                try {
                    Thread.sleep(RECONNECT_DELAY_MS);
                } catch (InterruptedException ignored) {}
            }
        }
    }

    public void connect() throws Exception {
        socket = new Socket(HOST, PORT);
        in = socket.getInputStream();
        out = socket.getOutputStream();

        // Handshake
        String hello = "{\"op\":\"hello\",\"protocol\":\"NT4\",\"version\":0}\n";
        out.write(hello.getBytes(StandardCharsets.UTF_8));
        out.flush();

        // Subscribe to the correct Limelight 3A topics
        subscribe("limelight/limelight/tx");
        subscribe("limelight/limelight/ty");
        subscribe("limelight/limelight/ta");

        System.out.println("[Limelight] Connected and subscribed.");
    }

    public void subscribe(String topic) throws Exception {
        String msg = String.format("{\"op\":\"subscribe\",\"topics\":[\"/%s\"]}\n", topic);
        out.write(msg.getBytes(StandardCharsets.UTF_8));
        out.flush();
    }

    public void listenLoop() throws Exception {
        byte[] buffer = new byte[4096];
        while (running && socket != null && !socket.isClosed()) {
            int len = in.read(buffer);
            if (len > 0) {
                String msg = new String(buffer, 0, len, StandardCharsets.UTF_8);
                parseMessage(msg);
            } else {
                throw new Exception("Socket closed or no data");
            }
        }
    }

    public void parseMessage(String msg) {
        try {
            JSONObject json = new JSONObject(msg);

            if (json.has("topic")) {
                String topic = json.getString("topic");

                // Debug: print unknown topic names
                if (tx == null && ty == null && ta == null) {
                    System.out.println("[Limelight topic] " + topic);
                }

                if (json.has("value")) {
                    double value = json.getJSONArray("value").getDouble(0);

                    if (topic.contains("tx")) {tx = value;}
                    else if (topic.contains("ty")) {ty = value;}
                    else if (topic.contains("ta")) {ta = value;}
                    else {return;}
                }
            }
        } catch (Exception e) {
            System.out.println("[Parse error] " + e.getMessage());
        }
    }


    public Double getTx() { return tx; }

    public Double getTy() { return ty; }
    public Double getTa() { return ta; }

    public void stop() {
        running = false;
        closeSocket();
        try {
            if (listenerThread != null) listenerThread.join(200);
        } catch (InterruptedException ignored) {}
    }

    public void closeSocket() {
        try {
            if (socket != null) socket.close();
        } catch (Exception ignored) {}
        socket = null;
        in = null;
        out = null;
    }
}