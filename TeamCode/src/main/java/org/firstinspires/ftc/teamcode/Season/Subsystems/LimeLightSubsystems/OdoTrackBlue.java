package org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems;

import static org.firstinspires.ftc.teamcode.Season.Auto.Tuning.follower;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * RED ALLIANCE - Odometry-Only Localization System
 * Goal position: (121, 121) inches
 *
 * POSITION (X, Y, Heading): 100% from GoBilda Pinpoint odometry
 *
 * RELOCALIZE FEATURE:
 * - relocalize() now simply pushes current odometry pose into Pedro follower
 */

public class OdoTrackBlue {

    // ========== HARDWARE ==========
    private GoBildaPinpointDriver pinpoint;
    private Telemetry telemetry;

    // ========== POSE TRACKING ==========
    // Position always equals odometry position
    private double x = 0;          // Field X position (inches)
    private double y = 0;          // Field Y position (inches)
    private double heading = 0;    // Field heading (degrees)

    // ========== BLUE ALLIANCE TARGET ==========
    private static final double BLUE_GOAL_X = -121.0;   // Blue goal X position (inches)
    private static final double BLUE_GOAL_Y = 121.0;   // Blue goal Y position (inches)

    // ========== PINPOINT CONFIGURATION ==========
    // TUNE THESE FOR YOUR ROBOT
    private double xOffset = -135;  // mm, forward offset from center
    private double yOffset = 60;    // mm, left offset from center

    /**
     * Initialize with default starting position at (0, 0, 0)
     * IMPORTANT: Call setPose() immediately after construction to set your actual starting position!
     */
    public OdoTrackBlue(HardwareMap hardwareMap, Telemetry telemetry) {
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
    public OdoTrackBlue(HardwareMap hardwareMap, Telemetry telemetry,
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

        telemetry.addLine("========================================");
        telemetry.addLine("ODOMETRY-ONLY LOCALIZATION");
        telemetry.addLine("========================================");
        telemetry.addData("Starting Position", "%.1f, %.1f @ %.1f°", x, y, heading);
        telemetry.addLine();
        telemetry.addLine("Position source: ODOMETRY ONLY");
        telemetry.addLine();
        telemetry.addLine("⚠ Make sure to call setPose() if this");
        telemetry.addLine("  starting position is wrong!");
        telemetry.update();
    }

    /**
     * Update the subsystem - call this every loop
     * Position is ALWAYS from odometry
     */
    public void update() {
        // ========== UPDATE PINPOINT ODOMETRY ==========
        pinpoint.update();

        Pose2D pinpointPose = pinpoint.getPosition();
        x = pinpointPose.getX(DistanceUnit.INCH);
        y = pinpointPose.getY(DistanceUnit.INCH);
        heading = pinpointPose.getHeading(AngleUnit.DEGREES);
    }

    /**
     * Relocalize using current odometry and update Pedro Pathfinding follower
     * This updates the Pedro follower position using odometry pose
     */
    public void relocalize() {
        Pose pedroPose = new Pose(x, y, Math.toRadians(heading), FTCCoordinates.INSTANCE)
                .getAsCoordinateSystem(PedroCoordinates.INSTANCE);
        follower.setPose(pedroPose);
    }

    /**
     * Reset position and IMU to (0, 0, 0)
     */
    public void reset() {
        pinpoint.resetPosAndIMU();
        x = 0;
        y = 0;
        heading = 0;
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

    public double getX() { return x; }
    public double getY() { return y; }
    public double getHeading() { return heading; }

    public double getOdometryX() { return x; }
    public double getOdometryY() { return y; }
    public double getOdometryHeading() { return heading; }

    // ========== GOAL CALCULATIONS ==========

    /**
     * Calculate robot-relative angle to RED goal
     * Uses ODOMETRY HEADING
     * @return Angle in degrees (-180 to 180)
     */
    public double getAngleToGoal() {
        return getAngleToGoalOdometry();
    }

    /**
     * Calculate angle to goal using ODOMETRY HEADING
     * @return Angle in degrees (-180 to 180)
     */
    public double getAngleToGoalOdometry() {
        return calculateAngleToTarget(BLUE_GOAL_X, BLUE_GOAL_Y);
    }

    /**
     * Calculate distance to RED goal
     * @return Distance in inches
     */
    public double getDistanceToGoal() {
        return calculateDistance(BLUE_GOAL_X, BLUE_GOAL_Y);
    }

    /**
     * Calculate robot-relative angle to target using ODOMETRY HEADING
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

    // ========== TELEMETRY ==========

    public void displayTelemetry() {
        telemetry.addLine("========== BLUE ALLIANCE (ODOMETRY ONLY) ==========");
        telemetry.addData("Position", "%.1f, %.1f @ %.1f°", x, y, heading);
        telemetry.addLine();

        telemetry.addLine("========== BLUE GOAL (-121, 121) ==========");
        telemetry.addData("Angle to Goal", "%.1f°", getAngleToGoal());
        telemetry.addData("Distance", "%.1f inches", getDistanceToGoal());
        telemetry.addLine();

        telemetry.addLine("========== LOCALIZATION ==========");
        telemetry.addLine("Position (X, Y, Heading): ODOMETRY ONLY");
    }

    public void displayMinimalTelemetry() {
        telemetry.addLine("========== BLUE ALLIANCE ==========");
        telemetry.addData("Position", "%.1f, %.1f @ %.1f°", x, y, heading);
        telemetry.addData("Source", "Odometry Only");
        telemetry.addLine();
        telemetry.addData("→ Goal", "%.1f° @ %.1f\"", getAngleToGoal(), getDistanceToGoal());
    }

    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    public void stop() {
        // Nothing to stop for odometry-only
    }
}
