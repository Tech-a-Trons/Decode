package org.firstinspires.ftc.teamcode.Personal.Pranav.ObjectDetection.Limelight;

import java.io.InputStream;
import java.io.OutputStream;
import java.net.Socket;
import java.nio.charset.StandardCharsets;

public class LimelightExtractor {
    private static final String HOST = "limelight.local";  // Or use "192.168.43.11"
    private static final int PORT = 5810;

    private Socket socket;
    private InputStream in;
    private OutputStream out;

    public boolean connect() {
        try {
            socket = new Socket(HOST, PORT);
            in = socket.getInputStream();
            out = socket.getOutputStream();

            // Send NT4 HELLO handshake
            String hello = "{\"op\":\"hello\",\"protocol\":\"NT4\",\"version\":0}\n";
            out.write(hello.getBytes(StandardCharsets.UTF_8));
            out.flush();

            return true;
        } catch (Exception e) {
            e.printStackTrace();
            return false;
        }
    }

    public void subscribe(String topic) {
        try {
            String msg = String.format("{\"op\":\"subscribe\",\"topics\":[\"/%s\"]}\n", topic);
            out.write(msg.getBytes(StandardCharsets.UTF_8));
            out.flush();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public String readMessage() {
        try {
            byte[] buffer = new byte[2048];
            int len = in.read(buffer);
            if (len > 0) {
                return new String(buffer, 0, len, StandardCharsets.UTF_8);
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
        return null;
    }

    public void close() {
        try {
            socket.close();
        } catch (Exception ignored) {}
    }
}
