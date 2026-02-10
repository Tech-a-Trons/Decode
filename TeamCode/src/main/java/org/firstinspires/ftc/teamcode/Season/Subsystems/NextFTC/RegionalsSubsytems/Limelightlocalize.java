//package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems;
//
//import com.pedropathing.localization.Localizer;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.math.Vector;
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.LLResultTypes;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
//
//public class LocalizationSubsystem {
//
//    private final Localizer localizer;
//    private final Limelight3A limelight;
//    private final boolean isBlueAlliance;
//
//    // Tuning Parameters
//    private static final double MAX_AMBIGUITY = 0.20;
////    private static final double MAX_TAG_DISTANCE = 3.5;
//    private static final double MIN_VISION_WEIGHT = 0.15;
//    private static final double MAX_VISION_WEIGHT = 0.40;
//    private static final double VISION_WEIGHT_HEADING = 0.25;
//    private static final double MAX_POSE_JUMP = 0.5;
//    private static final double MIN_TAG_AREA = 0.1; // Minimum tag area percentage
//
//    // State tracking
//    private Pose lastVisionPose = null;
//    private long lastVisionUpdateMs = 0;
//    private int consecutiveVisionUpdates = 0;
//    private int totalVisionUpdates = 0;
//    private int rejectedUpdates = 0;
//
//    public LocalizationSubsystem(HardwareMap hardwareMap, boolean isBlueAlliance) {
//        this.isBlueAlliance = isBlueAlliance;
//        this.localizer = new Localizer() {
//            @Override
//            public Pose getPose() {
//                return null;
//            }
//
//            @Override
//            public Pose getVelocity() {
//                return null;
//            }
//
//            @Override
//            public Vector getVelocityVector() {
//                return null;
//            }
//
//            @Override
//            public void setStartPose(Pose setStart) {
//
//            }
//
//            @Override
//            public void setPose(Pose setPose) {
//
//            }
//
//            @Override
//            public void update() {
//
//            }
//
//            @Override
//            public double getTotalHeading() {
//                return 0;
//            }
//
//            @Override
//            public double getForwardMultiplier() {
//                return 0;
//            }
//
//            @Override
//            public double getLateralMultiplier() {
//                return 0;
//            }
//
//            @Override
//            public double getTurningMultiplier() {
//                return 0;
//            }
//
//            @Override
//            public void resetIMU() throws InterruptedException {
//
//            }
//
//            @Override
//            public double getIMUHeading() {
//                return 0;
//            }
//
//            @Override
//            public boolean isNAN() {
//                return false;
//            }
//        };
//
//        // Initialize Limelight
//        this.limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        limelight.pipelineSwitch(0); // Use pipeline 0 (AprilTag pipeline)
//        limelight.start();
//    }
//
//    /**
//     * Call this EVERY loop
//     */
//    public void update() {
//        localizer.update();
//        tryVisionUpdate();
//    }
//
//    /**
//     * Attempts to fuse vision data with odometry
//     */
//    private void tryVisionUpdate() {
//        LLResult result = limelight.getLatestResult();
//
//        if (result == null || !result.isValid()) {
//            consecutiveVisionUpdates = 0;
//            return;
//        }
//
//        if (result.getFiducialResults() == null || result.getFiducialResults().isEmpty()) {
//            consecutiveVisionUpdates = 0;
//            return;
//        }
//
//        // Get first fiducial result
//        LLResultTypes.FiducialResult fiducial = result.getFiducialResults().get(0);
//
//        // Try to get ambiguity - use different methods based on SDK version
//        double ambiguity = getTagAmbiguity(fiducial);
//
//        // If we can't get ambiguity, use tag area as quality metric
//        if (ambiguity < 0) {
//            double tagArea = getTagArea(fiducial);
//            if (tagArea < MIN_TAG_AREA) {
//                rejectedUpdates++;
//                consecutiveVisionUpdates = 0;
//                return;
//            }
//        } else if (ambiguity > MAX_AMBIGUITY) {
//            rejectedUpdates++;
//            consecutiveVisionUpdates = 0;
//            return;
//        }
//
//        // Get botpose
//        Pose3D botpose3D = result.getBotpose_MT2();
//        if (botpose3D == null) {
//            botpose3D = result.getBotpose();
//        }
//
//        if (botpose3D == null) {
//            consecutiveVisionUpdates = 0;
//            return;
//        }
//
//        double x = botpose3D.getPosition().x;
//        double y = botpose3D.getPosition().y;
//        double yaw = Math.toDegrees(botpose3D.getOrientation().getYaw());
//
//        if (x == 0 && y == 0 && yaw == 0) {
//            consecutiveVisionUpdates = 0;
//            return;
//        }
//
//        // Red alliance coordinate transform (if needed)
//        if (!isBlueAlliance) {
//            // Adjust these values based on your field setup
//            // Example: mirror across field center
//            // x = FIELD_WIDTH - x;
//            // y = FIELD_LENGTH - y;
//            // yaw = (yaw + 180) % 360;
//        }
//
//        double tagDistance = Math.hypot(x, y);
//        if (tagDistance > MAX_TAG_DISTANCE) {
//            rejectedUpdates++;
//            consecutiveVisionUpdates = 0;
//            return;
//        }
//
//        Pose visionPose = new Pose(x, y, Math.toRadians(yaw));
//
//        Pose currentPose = localizer.getPose();
//        double poseDelta = calculatePoseDelta(currentPose, visionPose);
//        if (poseDelta > MAX_POSE_JUMP && lastVisionPose != null) {
//            rejectedUpdates++;
//            consecutiveVisionUpdates = 0;
//            return;
//        }
//
//        fusePoses(visionPose, tagDistance, ambiguity >= 0 ? ambiguity : 0);
//
//        lastVisionPose = visionPose;
//        lastVisionUpdateMs = System.currentTimeMillis();
//        consecutiveVisionUpdates++;
//        totalVisionUpdates++;
//    }
//
//    /**
//     * Try multiple methods to get ambiguity based on SDK version
//     */
//    private double getTagAmbiguity(LLResultTypes.FiducialResult fiducial) {
//        try {
//            // Try newer SDK method
//            return fiducial.getAmbiguity();
//        } catch (Exception e1) {
//            try {
//                // Try alternate method name
//                return fiducial.ambiguity;
//            } catch (Exception e2) {
//                try {
//                    // Try getPoseAmbiguity
//                    return fiducial.getPoseAmbiguity();
//                } catch (Exception e3) {
//                    // Can't get ambiguity, return -1 to signal fallback
//                    return -1;
//                }
//            }
//        }
//    }
//
//    /**
//     * Get tag area as alternative quality metric
//     */
//    private double getTagArea(LLResultTypes.FiducialResult fiducial) {
//        try {
//            return fiducial.getTargetArea();
//        } catch (Exception e1) {
//            try {
//                return fiducial.ta;
//            } catch (Exception e2) {
//                return 1.0; // Default to passing if we can't check
//            }
//        }
//    }
//
//    /**
//     * Fuses vision pose with odometry using weighted average
//     */
//    private void fusePoses(Pose visionPose, double tagDistance, double ambiguity) {
//        Pose currentPose = localizer.getPose();
//        double visionWeight = calculateVisionWeight(tagDistance, ambiguity);
//
//        double fusedX = currentPose.getX() * (1 - visionWeight) +
//                visionPose.getX() * visionWeight;
//        double fusedY = currentPose.getY() * (1 - visionWeight) +
//                visionPose.getY() * visionWeight;
//        double fusedHeading = lerpAngle(
//                currentPose.getHeading(),
//                visionPose.getHeading(),
//                VISION_WEIGHT_HEADING
//        );
//
//        localizer.setPose(new Pose(fusedX, fusedY, fusedHeading));
//    }
//
//    /**
//     * Calculates vision weight based on tag distance and ambiguity
//     */
//    private double calculateVisionWeight(double tagDistance, double ambiguity) {
//        double distanceFactor = Math.max(0, 1 - (tagDistance / MAX_TAG_DISTANCE));
//        double ambiguityFactor = ambiguity >= 0 ?
//                Math.max(0, 1 - (ambiguity / MAX_AMBIGUITY)) :
//                0.5; // Neutral if ambiguity unknown
//        double confidenceFactor = Math.sqrt(distanceFactor * ambiguityFactor);
//        return MIN_VISION_WEIGHT + (MAX_VISION_WEIGHT - MIN_VISION_WEIGHT) * confidenceFactor;
//    }
//
//    /**
//     * Circular interpolation for angles
//     */
//    private double lerpAngle(double a, double b, double t) {
//        double diff = ((b - a + Math.PI) % (2 * Math.PI)) - Math.PI;
//        return a + diff * t;
//    }
//
//    /**
//     * Calculates euclidean distance between two poses
//     */
//    private double calculatePoseDelta(Pose a, Pose b) {
//        double dx = a.getX() - b.getX();
//        double dy = a.getY() - b.getY();
//        return Math.hypot(dx, dy);
//    }
//
//    public Pose getPose() {
//        return localizer.getPose();
//    }
//
//    public void setPose(Pose pose) {
//        localizer.setPose(pose);
//        lastVisionPose = null;
//    }
//
//    public boolean hasRecentVisionUpdate() {
//        return (System.currentTimeMillis() - lastVisionUpdateMs) < 500;
//    }
//
//    public int getConsecutiveVisionUpdates() {
//        return consecutiveVisionUpdates;
//    }
//
//    public void resetStats() {
//        totalVisionUpdates = 0;
//        rejectedUpdates = 0;
//        consecutiveVisionUpdates = 0;
//        lastVisionUpdateMs = 0;
//    }
//
//    /**
//     * Comprehensive telemetry for debugging
//     */
//    public void addTelemetry(Telemetry telemetry) {
//        Pose pose = getPose();
//
//        telemetry.addData("═══ POSE ═══", "");
//        telemetry.addData("X (m)", "%.3f", pose.getX());
//        telemetry.addData("Y (m)", "%.3f", pose.getY());
//        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(pose.getHeading()));
//
//        telemetry.addData("═══ LIMELIGHT ═══", "");
//
//        LLResult result = limelight.getLatestResult();
//        boolean hasTarget = result != null && result.isValid();
//
//        telemetry.addData("Target Valid", hasTarget);
//
//        if (hasTarget && result.getFiducialResults() != null && !result.getFiducialResults().isEmpty()) {
//            LLResultTypes.FiducialResult fiducial = result.getFiducialResults().get(0);
//
//            int tagId = fiducial.getFiducialId();
//            double ambiguity = getTagAmbiguity(fiducial);
//            double latency = result.getStaleness() + result.getCaptureLatency();
//
//            telemetry.addData("Tag ID", tagId);
//
//            if (ambiguity >= 0) {
//                telemetry.addData("Ambiguity", "%.3f %s", ambiguity,
//                        ambiguity < MAX_AMBIGUITY ? "✓" : "✗");
//            } else {
//                telemetry.addData("Ambiguity", "N/A (using area)");
//                telemetry.addData("Tag Area", "%.2f", getTagArea(fiducial));
//            }
//
//            telemetry.addData("Latency (ms)", "%.1f", latency);
//            telemetry.addData("Num Tags", result.getFiducialResults().size());
//        }
//
//        telemetry.addData("═══ FUSION ═══", "");
//        telemetry.addData("Vision Active", hasRecentVisionUpdate() ? "YES" : "NO");
//        telemetry.addData("Consecutive", consecutiveVisionUpdates);
//        telemetry.addData("Accepted", totalVisionUpdates);
//        telemetry.addData("Rejected", rejectedUpdates);
//
//        if (totalVisionUpdates + rejectedUpdates > 0) {
//            double acceptRate = 100.0 * totalVisionUpdates /
//                    (totalVisionUpdates + rejectedUpdates);
//            telemetry.addData("Accept Rate", "%.1f%%", acceptRate);
//        }
//    }
//
//    /**
//     * Minimal telemetry for competition
//     */
//    public void addCompactTelemetry(Telemetry telemetry) {
//        Pose pose = getPose();
//        telemetry.addData("Pose", "X:%.2f Y:%.2f H:%.0f°",
//                pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
//        telemetry.addData("Vision", hasRecentVisionUpdate() ? "✓" : "✗");
//    }
//
//    /**
//     * Stop the Limelight (call in OpMode stop())
//     */
//    public void stop() {
//        if (limelight != null) {
//            limelight.stop();
//        }
//    }
//}