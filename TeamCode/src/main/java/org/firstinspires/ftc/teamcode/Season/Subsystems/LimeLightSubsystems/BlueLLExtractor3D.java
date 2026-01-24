package org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

//Need to get Odo for Blue Goal
//CALLIBRATE LINES 54,55,82,83
public class BlueLLExtractor3D {

    // ========== HARDWARE ==========
    private GoBildaPinpointDriver pinpoint;
    private Limelight3A limelight;
    private Telemetry telemetry;

    // ========== POSE TRACKING ==========
    private double x = 0;          // Fused field X position (inches)
    private double y = 0;          // Fused field Y position (inches)
    private double heading = 0;    // Fused field heading (degrees)

    // Odometry-only pose
    private double odoX = 0;
    private double odoY = 0;
    private double odoHeading = 0;

    // Vision-only pose
    private double visionX = 0;
    private double visionY = 0;
    private double visionHeading = 0;

    // ========== FUSION PARAMETERS ==========
    private double visionWeight = 0.20;  // 0.0 = pure odo, 1.0 = pure vision
    private static final double MIN_TAG_DISTANCE = 12.0;  // Minimum distance to trust vision (inches)
    private static final double MAX_TAG_DISTANCE = 120.0; // Maximum distance to trust vision (inches)

    // ========== BLUE ALLIANCE TARGET ==========
    private static final double BLUE_GOAL_X = -121.0;   // Blue goal X position (odometry units)
    private static final double BLUE_GOAL_Y = 121.0;   // Blue goal Y position (odometry units)

    // ========== PINPOINT CONFIGURATION ==========
    // TUNE THESE FOR YOUR ROBOT
    private double xOffset = -84.0;  // mm, forward offset from center
    private double yOffset = -168.0; // mm, left offset from center

    // ========== LIMELIGHT CONFIGURATION ==========
    private static final int PIPELINE = 1;  // Pipeline 0 for BLUE alliance MegaTag2

    // ========== STATE ==========
    private boolean visionActive = false;
    private double visionConfidence = 0.0;
    private int numTagsDetected = 0;
    private double tagDistance = 0.0;

    /**
     * Initialize the BLUE Limelight Localization Subsystem
     * @param hardwareMap Hardware map from OpMode
     * @param telemetry Telemetry from OpMode
     */
    public BlueLLExtractor3D(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // ========== INITIALIZE PINPOINT ==========
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Set odometry pod positions (in mm from center of robot)
        pinpoint.setOffsets(xOffset, yOffset, DistanceUnit.MM);

        // Set encoder directions (TUNE THESE - may need to reverse)
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        // Reset and recalibrate
        pinpoint.resetPosAndIMU();

        // ========== INITIALIZE LIMELIGHT ==========
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");

        // Set to MegaTag2 pipeline (Pipeline 1 for BLUE)
        limelight.pipelineSwitch(PIPELINE);

        // Start polling for data
        limelight.start();

        telemetry.addLine("BLUE Limelight MegaTag2 Subsystem Initialized");
        telemetry.addData("Pipeline", PIPELINE);
        telemetry.addData("Pinpoint X Offset (mm)", xOffset);
        telemetry.addData("Pinpoint Y Offset (mm)", yOffset);
        telemetry.update();
    }

    /**
     * Update the subsystem - call this every loop
     */
    public void update() {
        // ========== UPDATE PINPOINT ODOMETRY ==========
        pinpoint.update();

        Pose2D pinpointPose = pinpoint.getPosition();
        odoX = pinpointPose.getX(DistanceUnit.INCH);
        odoY = pinpointPose.getY(DistanceUnit.INCH);
        odoHeading = pinpointPose.getHeading(AngleUnit.DEGREES);

        // ========== UPDATE LIMELIGHT VISION ==========
        LLResult result = limelight.getLatestResult();
        visionActive = false;

        if (result != null && result.isValid()) {
            // Get MegaTag2 botpose (field-relative robot position)
            Pose3D botpose = result.getBotpose_MT2();

            if (botpose != null) {
                // Convert meters to inches
                visionX = botpose.getPosition().x * 39.3701;
                visionY = botpose.getPosition().y * 39.3701;

                // Get heading from orientation (yaw)
                visionHeading = botpose.getOrientation().getYaw(AngleUnit.DEGREES);

                // Calculate distance to tags
                tagDistance = Math.sqrt(visionX * visionX + visionY * visionY);
                numTagsDetected = result.getFiducialResults().size();

                // Only use vision if within reasonable range
                if (tagDistance >= MIN_TAG_DISTANCE && tagDistance <= MAX_TAG_DISTANCE && numTagsDetected > 0) {
                    visionActive = true;

                    // Dynamic weighting based on distance (closer = more trust)
                    double distanceWeight = 1.0 - ((tagDistance - MIN_TAG_DISTANCE) / (MAX_TAG_DISTANCE - MIN_TAG_DISTANCE));
                    double dynamicWeight = visionWeight * distanceWeight;

                    visionConfidence = distanceWeight * 100.0;

                    // ========== FUSION: Weighted average ==========
                    x = (1 - dynamicWeight) * odoX + dynamicWeight * visionX;
                    y = (1 - dynamicWeight) * odoY + dynamicWeight * visionY;
                    heading = normalizeAngle((1 - dynamicWeight) * odoHeading + dynamicWeight * visionHeading);
                } else {
                    // Out of range or no tags, use pure odometry
                    x = odoX;
                    y = odoY;
                    heading = odoHeading;
                    visionConfidence = 0.0;
                }
            } else {
                // No botpose data, use pure odometry
                x = odoX;
                y = odoY;
                heading = odoHeading;
                visionConfidence = 0.0;
            }
        } else {
            // No valid Limelight data, use pure odometry
            x = odoX;
            y = odoY;
            heading = odoHeading;
            visionConfidence = 0.0;
        }
    }

    /**
     * Reset position and IMU
     */
    public void reset() {
        pinpoint.resetPosAndIMU();
        x = 0;
        y = 0;
        heading = 0;
    }

    /**
     * Set the robot pose to a known position
     * @param x X position (odometry units)
     * @param y Y position (odometry units)
     * @param headingDegrees Heading in degrees
     */
    public void setPose(double x, double y, double headingDegrees) {
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, headingDegrees));
        this.x = x;
        this.y = y;
        this.heading = headingDegrees;
    }

    /**
     * Set Pinpoint odometry offsets
     * @param xOffsetMM X offset in millimeters
     * @param yOffsetMM Y offset in millimeters
     */
    public void setPinpointOffsets(double xOffsetMM, double yOffsetMM) {
        this.xOffset = xOffsetMM;
        this.yOffset = yOffsetMM;
        pinpoint.setOffsets(xOffsetMM, yOffsetMM, DistanceUnit.MM);
    }

    /**
     * Set encoder directions for Pinpoint
     * @param xDirection X encoder direction
     * @param yDirection Y encoder direction
     */
    public void setEncoderDirections(GoBildaPinpointDriver.EncoderDirection xDirection,
                                     GoBildaPinpointDriver.EncoderDirection yDirection) {
        pinpoint.setEncoderDirections(xDirection, yDirection);
    }

    /**
     * Set vision fusion weight
     * @param weight 0.0 (pure odometry) to 1.0 (pure vision)
     */
    public void setVisionWeight(double weight) {
        this.visionWeight = Math.max(0.0, Math.min(1.0, weight));
    }

    /**
     * Get fused X position
     * @return X position in odometry units
     */
    public double getX() {
        return x;
    }

    /**
     * Get fused Y position
     * @return Y position in odometry units
     */
    public double getY() {
        return y;
    }

    /**
     * Get fused heading
     * @return Heading in degrees
     */
    public double getHeading() {
        return heading;
    }

    /**
     * Get odometry-only X position
     * @return Odometry X in odometry units
     */
    public double getOdometryX() {
        return odoX;
    }

    /**
     * Get odometry-only Y position
     * @return Odometry Y in odometry units
     */
    public double getOdometryY() {
        return odoY;
    }

    /**
     * Get odometry-only heading
     * @return Odometry heading in degrees
     */
    public double getOdometryHeading() {
        return odoHeading;
    }

    /**
     * Get vision X position
     * @return Vision X in odometry units
     */
    public double getVisionX() {
        return visionX;
    }

    /**
     * Get vision Y position
     * @return Vision Y in odometry units
     */
    public double getVisionY() {
        return visionY;
    }

    /**
     * Get vision heading
     * @return Vision heading in degrees
     */
    public double getVisionHeading() {
        return visionHeading;
    }

    /**
     * Check if vision is actively being used
     * @return true if vision is active and trusted
     */
    public boolean isVisionActive() {
        return visionActive;
    }

    /**
     * Get vision confidence percentage
     * @return Confidence 0-100%
     */
    public double getVisionConfidence() {
        return visionConfidence;
    }

    /**
     * Get number of AprilTags detected
     * @return Number of tags
     */
    public int getNumTagsDetected() {
        return numTagsDetected;
    }

    /**
     * Get X velocity from Pinpoint
     * @return X velocity in inches/second
     */
//    public double getVelocityX() {
//        return pinpoint.getVelocity().getX(DistanceUnit.INCH);
//    }
//
//    /**
//     * Get Y velocity from Pinpoint
//     * @return Y velocity in inches/second
//     */
//    public double getVelocityY() {
//        return pinpoint.getVelocity().getY(DistanceUnit.INCH);
//    }

    /**
     * Calculate robot-relative angle to BLUE goal
     * @return Angle in degrees
     */
    public double getAngleToGoal() {
        return calculateAngleToTarget(BLUE_GOAL_X, BLUE_GOAL_Y);
    }

    /**
     * Calculate distance to BLUE goal
     * @return Distance in odometry units
     */
    public double getDistanceToGoal() {
        return calculateDistance(BLUE_GOAL_X, BLUE_GOAL_Y);
    }

    /**
     * Calculate robot-relative angle to any target
     * @param targetX Target X position
     * @param targetY Target Y position
     * @return Angle in degrees
     */
    public double calculateAngleToTarget(double targetX, double targetY) {
        double deltaX = targetX - x;
        double deltaY = targetY - y;
        double angleToTarget = Math.toDegrees(Math.atan2(deltaY, deltaX));
        double robotRelative = angleToTarget - heading;
        return normalizeAngle(robotRelative);
    }

    /**
     * Calculate distance to any target
     * @param targetX Target X position
     * @param targetY Target Y position
     * @return Distance in odometry units
     */
    public double calculateDistance(double targetX, double targetY) {
        double deltaX = targetX - x;
        double deltaY = targetY - y;
        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }

    /**
     * Display comprehensive telemetry
     */
    public void displayTelemetry() {
        telemetry.addLine("========== BLUE ALLIANCE ==========");
        telemetry.addData("X Position", "%.1f", x);
        telemetry.addData("Y Position", "%.1f", y);
        telemetry.addData("Heading", "%.1f°", heading);
        telemetry.addLine();

        telemetry.addLine("========== ODOMETRY ==========");
        telemetry.addData("Odo X", "%.1f", odoX);
        telemetry.addData("Odo Y", "%.1f", odoY);
        telemetry.addData("Odo Heading", "%.1f°", odoHeading);
//        telemetry.addData("Velocity X", "%.1f/s", getVelocityX());
//        telemetry.addData("Velocity Y", "%.1f/s", getVelocityY());
        telemetry.addLine();

        telemetry.addLine("========== LIMELIGHT MEGATAG2 ==========");
        telemetry.addData("Pipeline", PIPELINE);
        telemetry.addData("Vision Active", visionActive ? "✓ YES" : "✗ NO");
        if (visionActive) {
            telemetry.addData("Vision X", "%.1f", visionX);
            telemetry.addData("Vision Y", "%.1f", visionY);
            telemetry.addData("Vision Heading", "%.1f°", visionHeading);
            telemetry.addData("Confidence", "%.0f%%", visionConfidence);
            telemetry.addData("Tag Distance", "%.1f", tagDistance);
        }
        telemetry.addData("Tags Detected", numTagsDetected);
        telemetry.addLine();

        telemetry.addLine("========== BLUE GOAL (-121, 121) ==========");
        telemetry.addData("Angle to Goal", "%.1f°", getAngleToGoal());
        telemetry.addData("Distance to Goal", "%.1f", getDistanceToGoal());
    }

    /**
     * Normalize angle to [-180, 180]
     */
    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    /**
     * Stop the Limelight
     */
    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }
}
