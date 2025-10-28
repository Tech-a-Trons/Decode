package org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

/**
 * FTC-Legal Limelight 3A extractor with background polling thread.
 * Includes AprilTag ID extraction using LLResultTypes.FiducialResult.
 */
public class ExperimentalSeasonLExtractor {

    private final double SMOOTHING_FACTOR = 0.2;
    private final long STALE_TIMEOUT_MS = 500; // 0.5s

    private Limelight3A limelight;
    private Telemetry opModeTelemetry;

    // Raw Limelight values
    private volatile Double tx = null;
    private volatile Double ty = null;
    private volatile Double ta = null;
    private volatile Integer tagId = null;

    // Smoothed / derived values
    private volatile double horizontalAngle = 0.0;
    private volatile double verticalAngle = 0.0;
    private volatile boolean targetVisible = false;

    private volatile String connectionStatus = "Not connected";
    private volatile long lastUpdateTime = 0;

    private Thread pollingThread;
    private volatile boolean running = false;

    public ExperimentalSeasonLExtractor(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(1);
        limelight.start();
    }

    public void setTelemetry(Telemetry telemetry) {
        this.opModeTelemetry = telemetry;
    }

    public void startReading() {
        if (running) return;
        running = true;

        pollingThread = new Thread(() -> {
            while (running && !Thread.currentThread().isInterrupted()) {
                try {
                    LLResult result = limelight.getLatestResult();
                    long now = System.currentTimeMillis();

                    if (result != null && result.isValid()) {
                        tx = result.getTx();
                        ty = result.getTy();
                        ta = result.getTa();
                        tagId = extractTagId(result);

                        lastUpdateTime = now;
                        connectionStatus = "Connected";
                    }

                    // Handle stale data
                    if (now - lastUpdateTime > STALE_TIMEOUT_MS) {
                        tx = null;
                        ty = null;
                        ta = null;
                        tagId = null;
                        targetVisible = false;
                        horizontalAngle = 0.0;
                        verticalAngle = 0.0;
                        connectionStatus = "No data (stale)";
                    } else {
                        targetVisible = ta != null && ta > 0.0;
                        horizontalAngle = smooth(horizontalAngle, tx != null ? tx : 0.0);
                        verticalAngle = smooth(verticalAngle, ty != null ? ty : 0.0);
                    }

                    Thread.sleep(10); // small delay to reduce CPU usage
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        });
        pollingThread.start();
    }

    /**
     * Extracts the first detected AprilTag ID from the LLResult.
     */
    private Integer extractTagId(LLResult result) {
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials != null && !fiducials.isEmpty()) {
            return fiducials.get(0).getFiducialId(); // ✅ correct SDK call
        }
        return null;
    }

    public void stopReading() {
        running = false;
        if (pollingThread != null) {
            pollingThread.interrupt();
        }
        limelight.stop();
        connectionStatus = "Stopped";
    }

    public void update() {
        if (opModeTelemetry != null) {
            addTelemetry(opModeTelemetry);
            opModeTelemetry.update();
        }
    }

    private double smooth(double oldVal, double newVal) {
        return oldVal * (1.0 - SMOOTHING_FACTOR) + newVal * SMOOTHING_FACTOR;
    }

    private void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Limelight Status", connectionStatus);
        telemetry.addData("Target Visible", targetVisible);
        telemetry.addData("tx", tx != null ? String.format("%.2f", tx) : "N/A");
        telemetry.addData("ty", ty != null ? String.format("%.2f", ty) : "N/A");
        telemetry.addData("ta", ta != null ? String.format("%.2f", ta) : "N/A");
        telemetry.addData("Tag ID", tagId != null ? tagId : "None");
        telemetry.addData("Horizontal Angle", String.format("%.2f", horizontalAngle));
        telemetry.addData("Vertical Angle", String.format("%.2f", verticalAngle));
    }

    // ------------------- GETTERS -------------------
    public Double getTx() { return tx; }
    public Double getTy() { return ty; }
    public Double getTa() { return ta; }
    public Integer getTagId() { return tagId; }
    public double getHorizontalAngle() { return horizontalAngle; }
    public double getVerticalAngle() { return verticalAngle; }
    public boolean isTargetVisible() { return targetVisible; }
    public String getStatus() { return connectionStatus; }
}