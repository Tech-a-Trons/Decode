package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems.RedExperimentalDistanceLExtractor;

import com.pedropathing.geometry.Pose;
import dev.nextftc.extensions.pedro.PedroComponent;

@Config
public class HybridTurretNoEncoder {
    private CRServo turretServo;
    private RedExperimentalDistanceLExtractor limelight;
    private Telemetry telemetry;

    // Target position (goal location)
    public static double TARGET_X = 121;
    public static double TARGET_Y = 121;

    // Hybrid control - uses odometry estimate for coarse, Limelight for fine
    public static double LIMELIGHT_ACTIVATION_ANGLE = 20.0; // Use LL when within 20° (estimated)
    public static double ODOMETRY_ALIGNMENT_TIME = 1.0;     // Seconds to align via odometry

    // Odometry-based open-loop control (timed rotation)
    public static double ODO_ROTATION_POWER = 0.4;
    public static double ODO_ROTATION_RATE = 60.0; // degrees/second (tune this!)

    // Limelight-based control (fine alignment)
    public static double LL_BASE_POWER = 0.03;
    public static double LL_kP = 0.006;
    public static double LL_MAX_POWER = 0.4;
    public static double LL_ALIGNMENT_THRESHOLD = 1.5;

    // State tracking
    private String currentMode = "IDLE"; // "SEEKING", "LIMELIGHT", "IDLE"
    private boolean isAligning = false;
    private long seekStartTime = 0;
    private double estimatedAngleError = 0;

    public HybridTurretNoEncoder(HardwareMap hardwareMap, RedExperimentalDistanceLExtractor limelight) {
        this.turretServo = hardwareMap.get(CRServo.class, "turret");
        this.limelight = limelight;
    }

    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    /**
     * Calculate angle error to goal using odometry
     */
    private double calculateOdometryAngleError() {
        Pose robotPose = PedroComponent.follower().getPose();

        double robotX = robotPose.getX();
        double robotY = robotPose.getY();
        double robotHeading = Math.toDegrees(robotPose.getHeading());

        // Calculate vector from robot to target
        double deltaX = TARGET_X - robotX;
        double deltaY = TARGET_Y - robotY;

        // Calculate absolute angle to target (field-centric)
        double angleToTarget = Math.toDegrees(Math.atan2(deltaY, deltaX));

        // Convert to robot-centric angle
        double turretAngle = angleToTarget - robotHeading;

        // Normalize angle to -180 to 180
        while (turretAngle > 180) turretAngle -= 360;
        while (turretAngle < -180) turretAngle += 360;

        return turretAngle;
    }

    /**
     * Limelight-based fine alignment
     */
    private double calculateLimelightPower() {
        Double tx = limelight.getTx();

        if (tx == null) {
            return 0.0;
        }

        // If within threshold, stop
        if (Math.abs(tx) <= LL_ALIGNMENT_THRESHOLD) {
            return 0.0;
        }

        // Calculate power based on error
        double power = LL_BASE_POWER + (LL_kP * Math.abs(tx));
        power = Math.min(power, LL_MAX_POWER);

        // Determine direction
        if (tx > LL_ALIGNMENT_THRESHOLD) {
            return -power; // Turn LEFT
        } else if (tx < -LL_ALIGNMENT_THRESHOLD) {
            return power;  // Turn RIGHT
        }

        return 0.0;
    }

    /**
     * Main hybrid alignment method - NO ENCODER NEEDED
     */
    public void align() {
        // Calculate estimated angle error from odometry
        estimatedAngleError = calculateOdometryAngleError();

        // Check if Limelight can see target
        boolean limelightHasTarget = limelight.getTx() != null && limelight.isTargetVisible();

        // DECISION LOGIC: Choose mode
        if (limelightHasTarget && Math.abs(estimatedAngleError) <= LIMELIGHT_ACTIVATION_ANGLE) {
            // LIMELIGHT MODE: Target visible and we're close enough
            currentMode = "LIMELIGHT";

            double power = calculateLimelightPower();

            if (power == 0.0) {
                // Aligned!
                stopTurret();
            } else {
                turretServo.setPower(power);
                isAligning = true;
            }

        } else {
            // SEEKING MODE: Use odometry estimate to get close
            currentMode = "SEEKING";
            isAligning = true;

            // Simple proportional control based on estimated error
            double power = Range.clip(
                    estimatedAngleError * 0.015,  // Simple P control
                    -ODO_ROTATION_POWER,
                    ODO_ROTATION_POWER
            );

            // Apply minimum power threshold
            if (Math.abs(power) > 0.001 && Math.abs(power) < 0.08) {
                power = Math.signum(power) * 0.08;
            }

            // If error is very small but LL still can't see, slowly sweep
            if (Math.abs(estimatedAngleError) < 5.0 && !limelightHasTarget) {
                power = 0.15 * Math.signum(estimatedAngleError); // Slow sweep
            }

            turretServo.setPower(power);
        }
    }

    /**
     * Close alignment mode
     */
    public void closeAlign() {
        double originalLLMax = LL_MAX_POWER;
        double originalOdoPower = ODO_ROTATION_POWER;

        LL_MAX_POWER = 0.3;
        ODO_ROTATION_POWER = 0.3;

        align();

        LL_MAX_POWER = originalLLMax;
        ODO_ROTATION_POWER = originalOdoPower;
    }

    /**
     * Far alignment mode
     */
    public void farAlign() {
        align();
    }

    /**
     * Stop turret
     */
    public void stopTurret() {
        turretServo.setPower(0);
        isAligning = false;
        currentMode = "IDLE";
    }

    /**
     * Reset state
     */
    public void resetPID() {
        seekStartTime = 0;
    }

    /**
     * Check if aligned
     */
    public boolean isAligned() {
        if (currentMode.equals("LIMELIGHT")) {
            Double tx = limelight.getTx();
            return tx != null && Math.abs(tx) <= LL_ALIGNMENT_THRESHOLD;
        } else if (currentMode.equals("SEEKING")) {
            // Consider aligned if LL can see and error is reasonable
            return limelight.isTargetVisible() && Math.abs(estimatedAngleError) <= 5.0;
        }
        return false;
    }

    /**
     * Check if aligning
     */
    public boolean isAligning() {
        return isAligning;
    }

    /**
     * Set target position
     */
    public void setTarget(double x, double y) {
        TARGET_X = x;
        TARGET_Y = y;
    }

    /**
     * Update telemetry
     */
    public void updateTelemetry() {
        if (telemetry != null) {
            Double tx = limelight.getTx();
            Pose pose = PedroComponent.follower().getPose();

            telemetry.addData("--- Hybrid Turret (No Encoder) ---", "");
            telemetry.addData("Mode", currentMode);
            telemetry.addData("Estimated Angle Error", String.format("%.2f°", estimatedAngleError));
            telemetry.addData("LL Target Visible", limelight.isTargetVisible());
            telemetry.addData("LL TX", tx != null ? String.format("%.2f°", tx) : "N/A");
            telemetry.addData("Is Aligned", isAligned());
            telemetry.addData("Robot Pose", String.format("(%.1f, %.1f, %.1f°)",
                    pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading())));
        }
    }

    /**
     * Get telemetry string
     */
    public String getTelemetry() {
        Double tx = limelight.getTx();

        return String.format(
                "Hybrid Turret (No Encoder):\n" +
                        "  Mode: %s\n" +
                        "  Est. Error: %.2f°\n" +
                        "  LL TX: %s\n" +
                        "  LL Visible: %b\n" +
                        "  Aligned: %b",
                currentMode,
                estimatedAngleError,
                tx != null ? String.format("%.2f°", tx) : "N/A",
                limelight.isTargetVisible(),
                isAligned()
        );
    }
}