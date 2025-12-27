//package org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems;
//
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import dev.nextftc.core.subsystems.Subsystem;
//
//public class RedTurretAlign implements Subsystem {
//
//    public static final RedTurretAlign INSTANCE = new RedTurretAlign();
//
//    public static double kP = 0.01;
//    public static double ALIGN_TOLERANCE_DEG = 1.0;
//
//    public static double SERVO_MIN = -1.0;
//    public static double SERVO_MAX = 1.0;
//
//    private Servo turret;
//    private double servoPosition = 0.0;
//
//    private RedExperimentalDistanceLExtractor limelight;
//    private boolean initialized = false;
//
//    private RedTurretAlign() {} // private for singleton
//
//    // Call this after OpMode starts
//    public void initHardware(HardwareMap hw) {
//        if (!initialized) {
//            turret = hw.get(Servo.class, "turret"); // change to your servo name
//            initialized = true;
//        }
//    }
//
//    public void setLimelight(RedExperimentalDistanceLExtractor ll) {
//        this.limelight = ll;
//    }
//
//    private boolean isRedTag(Integer tagId) {
//        return tagId != null && (tagId == 21 || tagId == 23);
//    }
//
//    @Override
//    public void periodic() {
//        if (!initialized || limelight == null) return;
//        if (!limelight.isTargetVisible()) return;
//
//        Integer tagId = limelight.getTagId();
//        if (!isRedTag(tagId)) return;
//
//        Double tx = limelight.getTx();
//        if (tx == null) return;
//
//        if (Math.abs(tx) <= ALIGN_TOLERANCE_DEG) return;
//
//        servoPosition -= kP * tx;
//        servoPosition = Math.max(SERVO_MIN, Math.min(SERVO_MAX, servoPosition));
//
//        turret.setPosition(0.5 + servoPosition);
//    }
//}
package org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.core.subsystems.Subsystem;

public class RedTurretAlign implements Subsystem {

    public static final RedTurretAlign INSTANCE = new RedTurretAlign();

    // Tuning parameters
    public static double kP = 0.015;  // Increased slightly for faster response
    public static double kD = 0.002;  // Derivative term to reduce oscillation
    public static double ALIGN_TOLERANCE_DEG = 2.0;  // Slightly larger tolerance
    public static double MAX_CORRECTION = 0.05;  // Maximum position change per cycle

    // Servo operates from 0.0 to 1.0
    public static double SERVO_CENTER = 0.5;
    public static double SERVO_RANGE = 0.4;  // ±0.4 from center = 0.1 to 0.9 range

    private Servo turret;
    private double targetPosition = SERVO_CENTER;
    private double lastError = 0.0;

    private RedExperimentalDistanceLExtractor ll;
    private boolean initialized = false;
    private boolean alignmentActive = false;

    public RedTurretAlign() {} // private for singleton

    /**
     * Initialize hardware - call this in onStartButtonPressed()
     */
    public void initHardware(HardwareMap hw) {
        if (!initialized) {
            turret = hw.get(Servo.class, "turret");
            turret.setPosition(SERVO_CENTER);
            targetPosition = SERVO_CENTER;
            initialized = true;
            ll = new RedExperimentalDistanceLExtractor(hw);
        }
    }

    /**
     * Set the limelight instance to use for vision data
     */
    public void setLimelight(RedExperimentalDistanceLExtractor ll) {
        this.ll = ll;
    }

    /**
     * Enable or disable active alignment
     */
    public void setAlignmentActive(boolean active) {
        this.alignmentActive = active;
        if (!active) {
            lastError = 0.0;  // Reset derivative term
        }
    }

    /**
     * Check if currently aligned to target
     */
    public boolean isAligned() {
        if (!initialized || ll == null) return false;
        Double tx = ll.getTx();
        return tx != null && Math.abs(tx) <= ALIGN_TOLERANCE_DEG;
    }

    /**
     * Check if tracking a valid red alliance tag
     */
    private boolean isRedTag(Integer tagId) {
        return tagId != null && (tagId == 21 || tagId == 23);
    }

    /**
     * Reset turret to center position
     */
    public void resetToCenter() {
        targetPosition = SERVO_CENTER;
        if (initialized) {
            turret.setPosition(SERVO_CENTER);
        }
        lastError = 0.0;
    }

    /**
     * Main periodic update - called automatically by NextFTC framework
     */
    @Override
    public void periodic() {

        if (!initialized || ll == null || !alignmentActive) {
            return;
        }

        // Check if we have a valid target
        if (!ll.isTargetVisible()) {
            return;
        }

        Integer tagId = ll.getTagId();
        if (!isRedTag(tagId)) {
            return;
        }

        Double tx = ll.getTx();
        if (tx == null) {
            return;
        }

        // Already aligned - no need to move
        if (Math.abs(tx) <= ALIGN_TOLERANCE_DEG) {
            lastError = tx;  // Update for next cycle
            return;
        }

        // Calculate PD correction
        double error = tx;  // Positive tx means target is to the right
        double derivative = error - lastError;
        double correction = (kP * error) + (kD * derivative);

        // Limit maximum correction per cycle to prevent jerky movement
        correction = Math.max(-MAX_CORRECTION, Math.min(MAX_CORRECTION, correction));

        // Update target position
        // Negative correction because positive tx means we need to move servo left
        targetPosition -= correction;

        // Clamp to valid servo range (center ± range)
        double minPos = SERVO_CENTER - SERVO_RANGE;
        double maxPos = SERVO_CENTER + SERVO_RANGE;
        targetPosition = Math.max(minPos, Math.min(maxPos, targetPosition));

        // Apply position to servo
        turret.setPosition(targetPosition);

        // Store error for next derivative calculation
        lastError = error;
    }

    // Getters for telemetry/debugging
    public double getCurrentPosition() {
        return targetPosition;
    }

    public double getLastError() {
        return lastError;
    }

    public boolean isInitialized() {
        return initialized;
    }
}