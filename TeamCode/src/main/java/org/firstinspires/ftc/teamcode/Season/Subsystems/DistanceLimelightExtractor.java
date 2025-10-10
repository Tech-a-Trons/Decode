package org.firstinspires.ftc.teamcode.Season.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

//This file is for testing - Pranav 10/10

/**
 * Competition-ready Limelight 3A extractor for Decode:
 * - Automatic distance calculation
 * - Supports multiple target heights and auto-calibrated mounting angles per target group
 * - Smoothed distance for TeleOp/Autonomous
 */
public class DistanceLimelightExtractor {

    private final double SMOOTHING_FACTOR = 0.2;
    private final long STALE_TIMEOUT_MS = 500;

    private Limelight3A limelight;
    private Telemetry opModeTelemetry;

    private volatile Double tx = null;
    private volatile Double ty = null;
    private volatile Double ta = null;
    private volatile Integer tagId = null;

    private volatile double horizontalAngle = 0.0;
    private volatile double verticalAngle = 0.0;
    private volatile double estimatedDistanceInches = 0.0;
    private volatile boolean targetVisible = false;

    private volatile String connectionStatus = "Not connected";
    private volatile long lastUpdateTime = 0;

    private Thread pollingThread;
    private volatile boolean running = false;

    // ---------------- Robot / Limelight config ----------------
    private double limelightHeightInches = 13.0;

    // Default mount angles per target group (will be auto-calibrated)
    private final Map<String, Double> mountAngles = new HashMap<>();
    private final Map<String, Boolean> calibrated = new HashMap<>();

    // Multi-height support: tag ID -> height in inches
    private final Map<Integer, Double> tagHeightsInches = new HashMap<>();
    private final double DEFAULT_TARGET_HEIGHT = 30.0;

    // Tag groups
    private final Map<Integer, String> tagGroups = new HashMap<>();

    public DistanceLimelightExtractor(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(1);
        limelight.start();

        // Example Decode 2025 tag setup:
        // Goal tags
        tagHeightsInches.put(20, 30.0);
        tagHeightsInches.put(24, 30.0);
        tagGroups.put(20, "Goal");
        tagGroups.put(24, "Goal");

        // Obelisk tags
        tagHeightsInches.put(21, 18.0);
        tagHeightsInches.put(22, 18.0);
        tagHeightsInches.put(23, 18.0);
        tagGroups.put(21, "Obelisk");
        tagGroups.put(22, "Obelisk");
        tagGroups.put(23, "Obelisk");

        // Initialize mount angles and calibration flags
        mountAngles.put("Goal", 4.0);
        mountAngles.put("Obelisk", 4.0);
        calibrated.put("Goal", false);
        calibrated.put("Obelisk", false);
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

                        targetVisible = ty != null && ta != null && ta > 0.0;

                        horizontalAngle = smooth(horizontalAngle, tx != null ? tx : 0.0);
                        verticalAngle = smooth(verticalAngle, ty != null ? ty : 0.0);

                        // Auto-calibrate per group
                        if (tagId != null && tagGroups.containsKey(tagId)) {
                            String group = tagGroups.get(tagId);
                            if (!calibrated.get(group)) {
                                autoCalibrate(group);
                            }
                        }

                        updateDistance();
                    }

                    if (now - lastUpdateTime > STALE_TIMEOUT_MS) {
                        tx = null;
                        ty = null;
                        ta = null;
                        tagId = null;
                        targetVisible = false;
                        horizontalAngle = 0.0;
                        verticalAngle = 0.0;
                        estimatedDistanceInches = 0.0;
                        connectionStatus = "No data (stale)";
                    }

                    Thread.sleep(10);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        });
        pollingThread.start();
    }

    private Integer extractTagId(LLResult result) {
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials != null && !fiducials.isEmpty()) {
            return fiducials.get(0).getFiducialId();
        }
        return null;
    }

    /**
     * Auto-calibrates mounting angle for the given target group.
     */
    private void autoCalibrate(String group) {
        if (ty != null && tagId != null) {
            double targetHeight = tagHeightsInches.getOrDefault(tagId, DEFAULT_TARGET_HEIGHT);
            double rawDistance = (targetHeight - limelightHeightInches) / Math.tan(Math.toRadians(mountAngles.get(group) + ty));
            double angleRad = Math.atan((targetHeight - limelightHeightInches) / rawDistance);
            mountAngles.put(group, Math.toDegrees(angleRad) - ty);
            calibrated.put(group, true);
            if (opModeTelemetry != null) {
                opModeTelemetry.addData("Auto-Calibrated " + group + " Angle", mountAngles.get(group));
                opModeTelemetry.update();
            }
        }
    }

    private void updateDistance() {
        if (ty != null) {
            double targetHeight = tagId != null && tagHeightsInches.containsKey(tagId)
                    ? tagHeightsInches.get(tagId)
                    : DEFAULT_TARGET_HEIGHT;

            String group = tagId != null && tagGroups.containsKey(tagId)
                    ? tagGroups.get(tagId)
                    : "Unknown";

            double angle = mountAngles.getOrDefault(group, 4.0);
            double angleRad = Math.toRadians(angle + ty);
            double rawDistance = (targetHeight - limelightHeightInches) / Math.tan(angleRad);
            estimatedDistanceInches = smooth(estimatedDistanceInches, rawDistance);
        } else {
            estimatedDistanceInches = smooth(estimatedDistanceInches, 0.0);
        }
    }

    public void stopReading() {
        running = false;
        if (pollingThread != null) pollingThread.interrupt();
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
        telemetry.addData("Smoothed Distance (in)",
                estimatedDistanceInches > 0 ? String.format("%.2f", estimatedDistanceInches) : "N/A");
        if (tagId != null && tagGroups.containsKey(tagId)) {
            telemetry.addData("Mount Angle (" + tagGroups.get(tagId) + ")", mountAngles.get(tagGroups.get(tagId)));
        }
    }

    // ------------------- GETTERS -------------------
    public Double getTx() { return tx; }
    public Double getTy() { return ty; }
    public Double getTa() { return ta; }
    public Integer getTagId() { return tagId; }
    public double getHorizontalAngle() { return horizontalAngle; }
    public double getVerticalAngle() { return verticalAngle; }
    public double getEstimatedDistanceInches() { return estimatedDistanceInches; }
    public boolean isTargetVisible() { return targetVisible; }
    public String getStatus() { return connectionStatus; }
}
