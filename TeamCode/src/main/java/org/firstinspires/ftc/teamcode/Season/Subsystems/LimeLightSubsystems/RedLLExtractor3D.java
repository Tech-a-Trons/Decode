package org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems;

import static org.firstinspires.ftc.teamcode.Season.Auto.Tuning.follower;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

/**
 * RED ALLIANCE - Hybrid Localization System
 * Uses Pipeline 4 for RED alliance MegaTag2
 * Goal position: (121, 121) inches
 *
 * HYBRID APPROACH:
 * - POSITION (X, Y): 100% from odometry (manually set with setPose())
 * - HEADING: Limelight MT2 when available, odometry as fallback
 * - Angle calculations use Limelight heading for better accuracy
 *
 * WHY HYBRID?
 * - Odometry position: Accurate when manually set
 * - Odometry heading: Can drift over time
 * - Limelight heading: More accurate for angle calculations
 *
 * RELOCALIZE FEATURE:
 * - relocalize() uses standard Limelight botpose (not MT2)
 * - Updates Pedro Pathfinding follower position
 * - Does NOT affect odometry position
 * - Use when you want to sync Pedro with vision
 */

//CALIBRATE LINES 58,59,88,89

public class RedLLExtractor3D {

    // ========== HARDWARE ==========
    private GoBildaPinpointDriver pinpoint;
    private Limelight3A limelight;
    private Telemetry telemetry;

    // ========== POSE TRACKING ==========
    // Position always equals odometry position (no fusion)
    private double x = 0;          // Field X position (inches) - from odometry
    private double y = 0;          // Field Y position (inches) - from odometry
    private double heading = 0;    // Field heading (degrees) - from odometry

    // Vision data (for debugging/reference only - NOT used for positioning)
    private double visionX = 0;
    private double visionY = 0;
    private double visionHeading = 0;

    // ========== RED ALLIANCE TARGET ==========
    private static final double RED_GOAL_X = 121.0;   // Red goal X position (inches)
    private static final double RED_GOAL_Y = 121.0;   // Red goal Y position (inches)

    // ========== PINPOINT CONFIGURATION ==========
    // TUNE THESE FOR YOUR ROBOT
    private double xOffset = -135;  // mm, forward offset from center
    private double yOffset = 60;    // mm, left offset from center

    // ========== LIMELIGHT CONFIGURATION ==========
    private static final int PIPELINE = 4;  // Pipeline 4 for RED alliance MegaTag2

    // ========== STATE ==========
    private int numTagsDetected = 0;
    private boolean limelightConnected = false;
    private boolean allowAutoInit = false;  // Auto-init DISABLED by default (odometry is source of truth)
    private boolean visionHasInitialized = false;  // Track if vision has ever corrected position
    private static final int MIN_TAGS_FOR_AUTO_INIT = 2;  // Require 2+ tags for auto-correction

    // ========== DEBUG STATE ==========
    private boolean hasValidResult = false;
    private boolean hasBotpose = false;
    private double rawVisionX_meters = 0.0;
    private double rawVisionY_meters = 0.0;
    private double visionOdoDiffX = 0.0;
    private double visionOdoDiffY = 0.0;
    private double visionOdoDiffHeading = 0.0;
    private String initStatus = "Auto-init disabled - using manual position";

    /**
     * Initialize with default starting position at (0, 0, 0)
     * IMPORTANT: Call setPose() immediately after construction to set your actual starting position!
     */
    public RedLLExtractor3D(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, 0, 0, 0);
    }

    /**
     * Initialize with a known starting position
     * @param hardwareMap Hardware map from OpMode
     * @param telemetry Telemetry from OpMode
     * @param startX Starting X position (inches)
     * @param startY Starting Y position (inches)
     * @param startHeading Starting heading (degrees)
     */
    public RedLLExtractor3D(HardwareMap hardwareMap, Telemetry telemetry,
                            double startX, double startY, double startHeading) {
        this.telemetry = telemetry;

        // ========== INITIALIZE PINPOINT ODOMETRY ==========
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Set odometry pod positions (in mm from center of robot)
        pinpoint.setOffsets(xOffset, yOffset, DistanceUnit.MM);

        // Set encoder directions - adjust these if readings are inverted
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );

        // Recalibrate IMU
        pinpoint.recalibrateIMU();

        // ========== SET INITIAL POSITION ==========
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, startX, startY,
                AngleUnit.DEGREES, startHeading));

        // CRITICAL: Update to register the position
        pinpoint.update();

        // Read back and verify
        Pose2D verifyPose = pinpoint.getPosition();
        x = verifyPose.getX(DistanceUnit.INCH);
        y = verifyPose.getY(DistanceUnit.INCH);
        heading = verifyPose.getHeading(AngleUnit.DEGREES);

        // ========== INITIALIZE LIMELIGHT (for debugging only) ==========
        try {
            limelight = hardwareMap.get(Limelight3A.class, "Limelight");
            limelight.pipelineSwitch(PIPELINE);
            limelight.start();
            limelightConnected = true;
        } catch (Exception e) {
            limelightConnected = false;
            telemetry.addLine("⚠ WARNING: Limelight not found - odometry-only mode");
        }

        telemetry.addLine("========================================");
        telemetry.addLine("ODOMETRY-ONLY LOCALIZATION");
        telemetry.addLine("========================================");
        telemetry.addData("Starting Position", "%.1f, %.1f @ %.1f°", x, y, heading);
        telemetry.addData("Limelight", limelightConnected ? "Connected (debug only)" : "Not found");
        telemetry.addLine();
        telemetry.addLine("Position source: ODOMETRY ONLY");
        telemetry.addLine("Vision data: REFERENCE ONLY");
        telemetry.addLine();
        telemetry.addLine("⚠ Make sure to call setPose() if this");
        telemetry.addLine("  starting position is wrong!");
        telemetry.update();
    }

    /**
     * Update the subsystem - call this every loop
     * Position is ALWAYS from odometry (no fusion)
     */
    public void update() {
        // ========== UPDATE PINPOINT ODOMETRY ==========
        pinpoint.update();

        Pose2D pinpointPose = pinpoint.getPosition();
        x = pinpointPose.getX(DistanceUnit.INCH);
        y = pinpointPose.getY(DistanceUnit.INCH);
        heading = pinpointPose.getHeading(AngleUnit.DEGREES);

        // ========== READ LIMELIGHT DATA (DEBUG ONLY) ==========
        if (!limelightConnected) {
            return; // Skip if Limelight not available
        }

        LLResult result = limelight.getLatestResult();
        hasValidResult = false;
        hasBotpose = false;

        if (result != null && result.isValid()) {
            hasValidResult = true;

            Pose3D botpose = result.getBotpose_MT2();

            if (botpose != null) {
                hasBotpose = true;

                // Store raw values
                rawVisionX_meters = botpose.getPosition().x;
                rawVisionY_meters = botpose.getPosition().y;

                // Convert to inches
                visionX = rawVisionX_meters * 39.3701;
                visionY = rawVisionY_meters * 39.3701;
                visionHeading = botpose.getOrientation().getYaw(AngleUnit.DEGREES);

                // Calculate differences (for debugging)
                visionOdoDiffX = visionX - x;
                visionOdoDiffY = visionY - y;
                visionOdoDiffHeading = normalizeAngle(visionHeading - heading);

                // Get number of tags
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                numTagsDetected = fiducials.size();

                // ========== AUTO-INITIALIZE FROM VISION (IF ENABLED) ==========
                if (!visionHasInitialized && allowAutoInit && numTagsDetected >= MIN_TAGS_FOR_AUTO_INIT) {
                    // Vision has good data - use it to correct odometry position
                    pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, visionX, visionY,
                            AngleUnit.DEGREES, visionHeading));

                    // Force update
                    pinpoint.update();
                    Pose2D updatedPose = pinpoint.getPosition();
                    x = updatedPose.getX(DistanceUnit.INCH);
                    y = updatedPose.getY(DistanceUnit.INCH);
                    heading = updatedPose.getHeading(AngleUnit.DEGREES);

                    visionHasInitialized = true;
                    allowAutoInit = false;  // Disable after first correction
                    initStatus = String.format("✓ Vision corrected position to (%.1f, %.1f) with %d tags",
                            visionX, visionY, numTagsDetected);

                    // Recalculate differences since we just updated position
                    visionOdoDiffX = 0.0;
                    visionOdoDiffY = 0.0;
                    visionOdoDiffHeading = 0.0;
                }
            } else {
                numTagsDetected = result.getFiducialResults().size();
            }
        } else {
            numTagsDetected = 0;
        }
    }

    /**
     * Relocalize using Limelight and update Pedro Pathfinding follower
     * Uses standard botpose (not MT2) for Pedro integration
     * This updates the Pedro follower position, NOT the odometry position
     */
    public void relocalize() {
        if (!limelightConnected) {
            return; // Skip if Limelight not available
        }

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose(); // Use regular botpose for Pedro
            if (botpose != null) {
                double xInches = botpose.getPosition().x * 39.3701;
                double yInches = botpose.getPosition().y * 39.3701;
                double yawRadians = Math.toRadians(botpose.getOrientation().getYaw());

                Pose pedroPose = new Pose(xInches, yInches, yawRadians, FTCCoordinates.INSTANCE)
                        .getAsCoordinateSystem(PedroCoordinates.INSTANCE);

                follower.setPose(pedroPose);
            }
        }
    }

    /**
     * Reset position and IMU to (0, 0, 0)
     * Also resets vision initialization status
     */
    public void reset() {
        pinpoint.resetPosAndIMU();
        x = 0;
        y = 0;
        heading = 0;
        visionHasInitialized = false;
        initStatus = allowAutoInit ? "Reset - waiting for vision" : "Reset to (0,0,0) - auto-init disabled";
    }

    /**
     * Set the robot pose to a known position
     * USE THIS to set your accurate starting position!
     *
     * Example: If starting in bottom-left corner facing forward:
     *   setPose(12, 12, 0);
     *
     * @param x X position (inches)
     * @param y Y position (inches)
     * @param headingDegrees Heading in degrees
     */
    public void setPose(double x, double y, double headingDegrees) {
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, headingDegrees));

        // Update to register
        pinpoint.update();

        // Read back verified position
        Pose2D verifyPose = pinpoint.getPosition();
        this.x = verifyPose.getX(DistanceUnit.INCH);
        this.y = verifyPose.getY(DistanceUnit.INCH);
        this.heading = verifyPose.getHeading(AngleUnit.DEGREES);

        initStatus = String.format("Manually set to (%.1f, %.1f, %.1f°)", this.x, this.y, this.heading);
    }

    /**
     * Disable automatic vision correction of odometry position
     * Use this when you trust your manual position more than vision
     * This is the DEFAULT behavior - vision will NOT change your position
     *
     * Example use cases:
     * - You know your exact starting position
     * - Vision is unreliable in your environment
     * - You want complete control over localization
     */
    public void disableAutoInit() {
        allowAutoInit = false;
        if (!visionHasInitialized) {
            initStatus = "Auto-init disabled - using manual position only";
        }
    }

    /**
     * Enable automatic vision correction of odometry position
     * Vision will correct your position ONCE when it gets good data (2+ tags)
     * Then auto-disables itself to prevent further corrections
     *
     * Example use cases:
     * - You suspect odometry has drifted significantly
     * - You want to sync position with vision once during a match
     * - Starting position is uncertain but vision is reliable
     *
     * IMPORTANT: This allows vision to CHANGE your odometry position!
     * Only use this if you trust the Limelight more than your current position
     */
    public void enableAutoInit() {
        if (!visionHasInitialized) {
            allowAutoInit = true;
            initStatus = "Auto-init enabled - waiting for good vision data";
        } else {
            initStatus = "Vision already corrected position - call forceReinitialize() first";
        }
    }

    /**
     * Force vision to be allowed to correct position again
     * Resets the initialization flag so enableAutoInit() will work again
     *
     * Use this when:
     * - You want to allow vision to correct position a second time
     * - Odometry has drifted significantly since last vision correction
     *
     * Usage: Call forceReinitialize(), then enableAutoInit()
     */
    public void forceReinitialize() {
        visionHasInitialized = false;
        allowAutoInit = false;
        initStatus = "Forced reinit - call enableAutoInit() to allow vision correction";
    }

    /**
     * Check if vision has ever corrected the odometry position
     * @return true if vision has corrected position at least once
     */
    public boolean hasVisionInitialized() {
        return visionHasInitialized;
    }

    /**
     * Check if auto-init is currently enabled
     * @return true if vision is allowed to correct position
     */
    public boolean isAutoInitEnabled() {
        return allowAutoInit;
    }

    /**
     * Get current initialization status
     * @return String describing auto-init state
     */
    public String getInitStatus() {
        return initStatus;
    }

    // ========== CONFIGURATION METHODS ==========

    public void setPinpointOffsets(double xOffsetMM, double yOffsetMM) {
        this.xOffset = xOffsetMM;
        this.yOffset = yOffsetMM;
        pinpoint.setOffsets(xOffsetMM, yOffsetMM, DistanceUnit.MM);
    }

    public void setEncoderDirections(GoBildaPinpointDriver.EncoderDirection xDirection,
                                     GoBildaPinpointDriver.EncoderDirection yDirection) {
        pinpoint.setEncoderDirections(xDirection, yDirection);
    }

    // ========== POSITION GETTERS ==========
    // These all return the same values since we only use odometry

    public double getX() { return x; }
    public double getY() { return y; }
    public double getHeading() { return heading; }

    public double getOdometryX() { return x; }
    public double getOdometryY() { return y; }
    public double getOdometryHeading() { return heading; }

    // Vision data (for reference only - NOT used for position)
    public double getVisionX() { return visionX; }
    public double getVisionY() { return visionY; }
    public double getVisionHeading() { return visionHeading; }

    // Vision status
    public boolean hasVisionData() { return hasBotpose; }
    public int getNumTagsDetected() { return numTagsDetected; }
    public boolean isLimelightConnected() { return limelightConnected; }

    // Get vision-odometry differences (for debugging drift)
    public double getVisionOdoDiffX() { return visionOdoDiffX; }
    public double getVisionOdoDiffY() { return visionOdoDiffY; }
    public double getVisionOdoDiffHeading() { return visionOdoDiffHeading; }

    // ========== GOAL CALCULATIONS ==========

    /**
     * Calculate robot-relative angle to RED goal
     * USES LIMELIGHT HEADING when available for better accuracy
     * Falls back to odometry if no vision data
     * @return Angle in degrees (-180 to 180)
     */
    public double getAngleToGoal() {
        return getAngleToGoalVision();  // Default to vision-based
    }

    /**
     * Calculate angle to goal using LIMELIGHT HEADING (more accurate)
     * Uses vision heading from MT2 to avoid odometry drift
     * @return Angle in degrees (-180 to 180), or odometry-based if no vision
     */
    public double getAngleToGoalVision() {
        if (hasBotpose) {
            // Use Limelight heading for accurate angle calculation
            return calculateAngleToTargetVision(RED_GOAL_X, RED_GOAL_Y);
        } else {
            // Fall back to odometry if no vision data
            return getAngleToGoalOdometry();
        }
    }

    /**
     * Calculate angle to goal using ODOMETRY HEADING (fallback only)
     * May be less accurate due to heading drift
     * @return Angle in degrees (-180 to 180)
     */
    public double getAngleToGoalOdometry() {
        return calculateAngleToTarget(RED_GOAL_X, RED_GOAL_Y);
    }

    /**
     * Calculate distance to RED goal
     * @return Distance in inches
     */
    public double getDistanceToGoal() {
        return calculateDistance(RED_GOAL_X, RED_GOAL_Y);
    }

    /**
     * Calculate robot-relative angle to target using VISION HEADING
     * Uses Limelight MT2 heading for accurate angle calculation
     * @param targetX Target X position
     * @param targetY Target Y position
     * @return Angle in degrees (-180 to 180)
     */
    public double calculateAngleToTargetVision(double targetX, double targetY) {
        double deltaX = targetX - x;
        double deltaY = targetY - y;
        double angleToTarget = Math.toDegrees(Math.atan2(deltaY, deltaX));
        double robotRelative = angleToTarget - visionHeading;  // Use VISION heading
        return normalizeAngle(robotRelative);
    }

    /**
     * Calculate robot-relative angle to target using ODOMETRY HEADING
     * May drift over time - prefer calculateAngleToTargetVision when possible
     * @param targetX Target X position
     * @param targetY Target Y position
     * @return Angle in degrees (-180 to 180)
     */
    public double calculateAngleToTarget(double targetX, double targetY) {
        double deltaX = targetX - x;
        double deltaY = targetY - y;
        double angleToTarget = Math.toDegrees(Math.atan2(deltaY, deltaX));
        double robotRelative = angleToTarget - heading;  // Use ODOMETRY heading
        return normalizeAngle(robotRelative);
    }

    /**
     * Calculate distance to any target
     * @param targetX Target X position
     * @param targetY Target Y position
     * @return Distance in inches
     */
    public double calculateDistance(double targetX, double targetY) {
        double deltaX = targetX - x;
        double deltaY = targetY - y;
        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }

    /**
     * Get the current heading being used for angle calculations
     * @return "Vision" if using Limelight heading, "Odometry" if fallback
     */
    public String getAngleSource() {
        return hasBotpose ? "Vision (MT2)" : "Odometry (fallback)";
    }

    // ========== TELEMETRY ==========

    public void displayTelemetry() {
        telemetry.addLine("========== RED ALLIANCE (HYBRID LOCALIZATION) ==========");
        telemetry.addData("Position", "%.1f, %.1f @ %.1f°", x, y, heading);
        telemetry.addData("Init Status", initStatus);
        telemetry.addData("Auto-Init", allowAutoInit ? "ENABLED" : "DISABLED");
        telemetry.addLine();

        telemetry.addLine("========== RED GOAL (121, 121) ==========");
        telemetry.addData("Angle to Goal", "%.1f° (%s)", getAngleToGoal(), getAngleSource());
        telemetry.addData("Distance", "%.1f inches", getDistanceToGoal());
        telemetry.addLine();

        // Vision debugging data
        if (limelightConnected) {
            telemetry.addLine("========== VISION DATA (MT2 - REFERENCE) ==========");

            if (hasBotpose) {
                telemetry.addData("Vision Position", "%.1f, %.1f @ %.1f°", visionX, visionY, visionHeading);
                telemetry.addData("Odometry Heading", "%.1f°", heading);
                telemetry.addData("Tags Detected", numTagsDetected);
                telemetry.addLine();

                // Show differences
                telemetry.addLine("--- Vision vs Odometry Difference ---");
                telemetry.addData("ΔX", "%.1f inches", visionOdoDiffX);
                telemetry.addData("ΔY", "%.1f inches", visionOdoDiffY);
                telemetry.addData("ΔHeading", "%.1f°", visionOdoDiffHeading);

                double totalDiff = Math.sqrt(visionOdoDiffX * visionOdoDiffX + visionOdoDiffY * visionOdoDiffY);
                telemetry.addData("Total Position Diff", "%.1f inches", totalDiff);

                telemetry.addLine();
                telemetry.addLine("✓ Using VISION HEADING for angle to goal");

                if (totalDiff > 12.0) {
                    telemetry.addLine("⚠ WARNING: Large vision/odo difference!");
                    telemetry.addLine("  Check if odometry is drifting");
                    telemetry.addLine("  Or call relocalize() to sync Pedro");
                }
            } else if (hasValidResult) {
                telemetry.addData("Vision Status", "Tags detected but no botpose");
                telemetry.addData("Tags", numTagsDetected);
                telemetry.addLine("⚠ Using ODOMETRY HEADING (no vision data)");
            } else {
                telemetry.addData("Vision Status", "No data");
                telemetry.addLine("⚠ Using ODOMETRY HEADING (no vision data)");
            }
        } else {
            telemetry.addLine("Limelight: Not connected");
            telemetry.addLine("⚠ Using ODOMETRY HEADING (no Limelight)");
        }

        telemetry.addLine();
        telemetry.addLine("========== HYBRID LOCALIZATION ==========");
        telemetry.addLine("Position (X, Y): ODOMETRY ONLY");
        telemetry.addLine("Heading (angles): VISION MT2 (when available)");
        telemetry.addLine("relocalize(): Updates Pedro follower");
    }

    public void displayMinimalTelemetry() {
        telemetry.addLine("========== RED ALLIANCE ==========");
        telemetry.addData("Position", "%.1f, %.1f @ %.1f°", x, y, heading);
        telemetry.addData("Source", "Odometry Only");

        if (hasBotpose) {
            double diff = Math.sqrt(visionOdoDiffX * visionOdoDiffX + visionOdoDiffY * visionOdoDiffY);
            telemetry.addData("Vision Diff", "%.1f\" (%d tags)", diff, numTagsDetected);
        }

        telemetry.addLine();
        telemetry.addData("→ Goal", "%.1f° @ %.1f\"", getAngleToGoal(), getDistanceToGoal());
    }

    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }
}