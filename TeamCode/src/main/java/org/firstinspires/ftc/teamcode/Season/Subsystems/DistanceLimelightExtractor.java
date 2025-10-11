package org.firstinspires.ftc.teamcode.Season.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

public class DistanceLimelightExtractor {

    private final Limelight3A limelight;
    private final Telemetry telemetry;

    // --- Hardcoded robot-specific constants (in inches & degrees) ---
    private final double LIMELIGHT_HEIGHT_INCHES = 13.0;
    private double limelightMountAngle = 3.0;       // degrees
    private double currentTargetHeightMeters = 0.762; // default 30 inches converted to meters

    // smoothing buffers
    private final LinkedList<Double> txSamples = new LinkedList<>();
    private final LinkedList<Double> tySamples = new LinkedList<>();
    private final LinkedList<Double> taSamples = new LinkedList<>();
    private final LinkedList<Double> distanceSamples = new LinkedList<>();
    private static final int SMOOTHING_WINDOW = 5;

    // latest values
    private double tx = 0, ty = 0, ta = 0, distanceMeters = 0;
    private boolean targetVisible = false;

    // pose values
    private double poseX = 0, poseY = 0, poseZ = 0;
    private double poseRoll = 0, posePitch = 0, poseYaw = 0;

    // fallback calibration for ta-based distance
    private double knownTagArea = 5.0;       // % area at 1m, adjust
    private double knownTagDistanceMeters = 0.991; // 39 inches converted to meters

    // map of tag IDs to heights (in inches)
    private final Map<Integer, Double> tagHeightsInches = new HashMap<>();

    /** Constructor - only need Limelight object and telemetry now */
    public DistanceLimelightExtractor(Limelight3A limelight, Telemetry telemetry) {
        this.limelight = limelight;
        this.telemetry = telemetry;

        // default target height in meters (30 inches)
        this.currentTargetHeightMeters = inchesToMeters(30.0);
    }

    /** Add mapping from tag ID to height (in inches) */
    public void addTagHeight(int tagId, double heightInches) {
        tagHeightsInches.put(tagId, heightInches);
    }

    /** Update readings each frame */
    public void update() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            tx = result.getTx();
            ty = result.getTy();
            ta = result.getTa();
            targetVisible = ta > 0;

            distanceMeters = 0;

            // check detected fiducials
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials != null && !fiducials.isEmpty()) {
                int tagId = fiducials.get(0).getFiducialId(); // get first detected tag
                if (tagHeightsInches.containsKey(tagId)) {
                    currentTargetHeightMeters = inchesToMeters(tagHeightsInches.get(tagId));
                }
            }


            // get Botpose safely
            Object botposeObj = result.getBotpose();
            if (botposeObj instanceof double[]) {
                double[] pose = (double[]) botposeObj;
                if (pose.length >= 6) {
                    poseX = pose[0];
                    poseY = pose[1];
                    poseZ = pose[2];
                    poseRoll = pose[3];
                    posePitch = pose[4];
                    poseYaw = pose[5];
                    distanceMeters = poseZ;
                }
            }

            // fallback using ta
            if (distanceMeters <= 0 && ta > 0) {
                distanceMeters = knownTagDistanceMeters * Math.sqrt(knownTagArea / ta);
                poseX = poseY = poseRoll = posePitch = poseYaw = 0;
            }

            addSample(txSamples, tx);
            addSample(tySamples, ty);
            addSample(taSamples, ta);
            addSample(distanceSamples, distanceMeters);
        } else {
            targetVisible = false;
        }

        if (telemetry != null) {
            telemetry.addData("Target Visible", targetVisible);
            telemetry.addData("tx", getSmoothed(txSamples));
            telemetry.addData("ty", getSmoothed(tySamples));
            telemetry.addData("ta", getSmoothed(taSamples));
            telemetry.addData("Distance (m)", String.format("%.2f", getSmoothed(distanceSamples)));
            telemetry.addData("Pose X", poseX);
            telemetry.addData("Pose Y", poseY);
            telemetry.addData("Pose Z", poseZ);
            telemetry.addData("Roll", poseRoll);
            telemetry.addData("Pitch", posePitch);
            telemetry.addData("Yaw", poseYaw);
        }
    }

    /** Dynamically change Limelight pipeline */
    public void setPipeline(int pipeline) {
        if (limelight != null) {
            limelight.pipelineSwitch(pipeline);
        }
    }

    /** Auto-calibrate mount angle using known target */
    public void calibrateMountAngle() {
        if (!targetVisible) return;
        limelightMountAngle = Math.toDegrees(Math.atan((currentTargetHeightMeters - inchesToMeters(LIMELIGHT_HEIGHT_INCHES)) / distanceMeters)) - ty;
        telemetry.addData("Calibrated Mount Angle", limelightMountAngle);
    }

    /** Set fallback constants for ta-based distance */
    public void setTaFallback(double knownTagAreaPercent, double knownTagDistanceInches) {
        this.knownTagArea = knownTagAreaPercent;
        this.knownTagDistanceMeters = inchesToMeters(knownTagDistanceInches);
    }

    private void addSample(LinkedList<Double> list, double value) {
        list.add(value);
        if (list.size() > SMOOTHING_WINDOW) list.removeFirst();
    }

    private double getSmoothed(LinkedList<Double> list) {
        if (list.isEmpty()) return 0;
        double sum = 0;
        for (double v : list) sum += v;
        return sum / list.size();
    }

    private double inchesToMeters(double inches) { return inches * 0.0254; }

    // --- Getters ---
    public boolean isTargetVisible() { return targetVisible; }
    public double getTx() { return getSmoothed(txSamples); }
    public double getTy() { return getSmoothed(tySamples); }
    public double getTa() { return getSmoothed(taSamples); }
    public double getDistanceMeters() { return getSmoothed(distanceSamples); }

    public double getPoseX() { return poseX; }
    public double getPoseY() { return poseY; }
    public double getPoseZ() { return poseZ; }
    public double getPoseRoll() { return poseRoll; }
    public double getPosePitch() { return posePitch; }
    public double getPoseYaw() { return poseYaw; }

    public double getMountAngle() { return limelightMountAngle; }
}
