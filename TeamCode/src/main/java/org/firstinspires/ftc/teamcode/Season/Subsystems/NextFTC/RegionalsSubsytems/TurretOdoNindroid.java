package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;

import dev.nextftc.core.subsystems.Subsystem;

/**
 * Fixed Turret Odometry Alignment System
 * - Works correctly with position servos (no fake feedback loop)
 * - Uses position-based PID instead of velocity-based
 * - Proper NextFTC subsystem integration
 * - Manual override support
 */
public class TurretOdoNindroid implements Subsystem {

    public static final TurretOdoNindroid INSTANCE = new TurretOdoNindroid();

    // ------------------ Hardware ------------------
    private Servo turretServo1;
    private Servo turretServo2;

    // ------------------ Robot Pose ------------------
    private double x = 0;
    private double y = 0;
    private double heading = 0;

    // ------------------ Target (Red Goal) ------------------
    public static double xt = 60;
    public static double yt = 60;

    // ------------------ Turret State ------------------
    private double targetAngleDeg = 0;          // Where turret should point (field-relative)
    private double commandedServoPos = 0.5;     // Last commanded servo position
    private double distanceToTarget = 0;
    private boolean autoAlignEnabled = true;

    // ------------------ Servo Limits ------------------
    public static double SERVO_MIN = 0.0;
    public static double SERVO_MAX = 1.0;
    public static double SERVO_CENTER = 0.5;

    // ========== PID CONSTANTS (TUNE THESE!) ==========
    // These now control servo POSITION directly, not velocity
    public static double kP = 0.008;            // Proportional gain (servo position per degree of error)
    public static double kI = 0.0001;           // Integral gain (start small)
    public static double kD = 0.0005;           // Derivative gain

    public static double TOLERANCE = 2.0;       // degrees (acceptable error)
    public static double MAX_POSITION_CHANGE = 0.05; // Max servo change per loop (rate limiting)

    // ========== PID STATE VARIABLES ==========
    private double lastError = 0;
    private double integral = 0;
    private double lastTime = 0;
    private boolean firstRun = true;

    // Telemetry
    private Telemetry telemetry;

    private TurretOdoNindroid() {
        // Singleton
    }

    // ------------------ Initialization ------------------

    public void onInit(HardwareMap hardwareMap) {
        turretServo1 = hardwareMap.get(Servo.class, "turretServo1");
        turretServo2 = hardwareMap.get(Servo.class, "turretServo2");

        // Start at center
        turretServo1.setPosition(SERVO_CENTER);
        turretServo2.setPosition(SERVO_CENTER);
        commandedServoPos = SERVO_CENTER;

        // Initialize time tracking
        lastTime = System.nanoTime() / 1e9;
        firstRun = true;

        integral = 0;
        lastError = 0;
    }

    // ------------------ Main Loop ------------------
    public void periodic() {
        // === 1. SAFETY CHECKS ===
        if (PedroComponent.follower() == null) {
            return;
        }

        Pose currentPose = PedroComponent.follower().getPose();
        if (currentPose == null) {
            return;
        }

        // === 2. UPDATE ROBOT POSE ===
        x = currentPose.getX();
        y = currentPose.getY();
        heading = Math.toDegrees(currentPose.getHeading());
        heading = normalizeAngle(heading);

        // === 3. CALCULATE TARGET ANGLE ===
        // Field-centric angle to goal
        double fieldAngleDeg = Math.toDegrees(Math.atan2(yt - y, xt - x));
        fieldAngleDeg = normalizeAngle(fieldAngleDeg);

        // Distance to target
        distanceToTarget = Math.hypot(xt - x, yt - y);

        // Robot-centric target angle (where turret should point relative to robot)
        targetAngleDeg = fieldAngleDeg - heading;
        targetAngleDeg = normalizeAngle(targetAngleDeg);

        // === 4. AUTO-ALIGNMENT (if enabled) ===
        if (!autoAlignEnabled) {
            // Manual mode - just update telemetry
            updateTelemetry();
            return;
        }

        // === 5. CALCULATE CURRENT TURRET ANGLE FROM SERVO POSITION ===
        // We use the last COMMANDED position, not a fake "read" from the servo
        double currentTurretAngle = servoToAngle(commandedServoPos);

        // === 6. CALCULATE ERROR (SHORTEST PATH) ===
        double error = targetAngleDeg - currentTurretAngle;
        error = normalizeAngle(error); // Wrap to [-180, 180] for shortest path

        // === 7. CALCULATE TIME DELTA ===
        double currentTime = System.nanoTime() / 1e9;
        double dt = currentTime - lastTime;

        if (firstRun || dt <= 0 || dt > 0.1) {
            dt = 0.02; // Default to 20ms if time is weird
            firstRun = false;
        }

        // === 8. PID CALCULATIONS (POSITION-BASED) ===

        // P term: Proportional to error
        double P_output = kP * error;

        // I term: Integral (accumulated error over time)
        integral += error * dt;

        // Anti-windup: Reset integral if we're close to target
        if (Math.abs(error) < TOLERANCE) {
            integral = 0;
        }

        // Clamp integral to prevent runaway
        integral = clamp(integral, -50, 50);
        double I_output = kI * integral;

        // D term: Derivative (rate of change of error)
        double derivative = (error - lastError) / dt;
        double D_output = kD * derivative;

        // === 9. COMBINE PID OUTPUTS ===
        // This is a POSITION adjustment, not velocity
        double positionAdjustment = P_output + I_output + D_output;

        // Rate limit the position change for smooth motion
        positionAdjustment = clamp(positionAdjustment, -MAX_POSITION_CHANGE, MAX_POSITION_CHANGE);

        // === 10. UPDATE COMMANDED SERVO POSITION ===
        double newServoPos = commandedServoPos + positionAdjustment;
        newServoPos = clamp(newServoPos, SERVO_MIN, SERVO_MAX);

        // Apply to both servos
        turretServo1.setPosition(newServoPos);
        turretServo2.setPosition(newServoPos);

        // Update our internal state
        commandedServoPos = newServoPos;

        // === 11. UPDATE STATE FOR NEXT LOOP ===
        lastError = error;
        lastTime = currentTime;

        // === 12. TELEMETRY ===
        updateTelemetry();
    }

    // ------------------ Helper Functions ------------------

    /**
     * Convert angle in degrees to servo position (0.0 to 1.0)
     * Uses your custom inverted mapping
     */
    private double angleToServo(double angleDeg) {
        angleDeg = normalizeAngle(angleDeg);
        // Your formula: pos = 1.0 - ((angle + 180) / 360)
        double pos = 1.0 - ((angleDeg + 180.0) / 360.0);
        return clamp(pos, SERVO_MIN, SERVO_MAX);
    }

    /**
     * Convert servo position (0.0 to 1.0) back to angle in degrees
     * Reverse of angleToServo()
     */
    private double servoToAngle(double servoPos) {
        // Reverse the formula:
        // pos = 1 - (angle + 180)/360
        // angle = 360 * (1 - pos) - 180
        double angle = 360.0 * (1.0 - servoPos) - 180.0;
        return normalizeAngle(angle);
    }

    /**
     * Normalize angle to -180 to +180 range
     */
    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    /**
     * Clamp value between min and max
     */
    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    /**
     * Update telemetry if available
     */
    private void updateTelemetry() {
        if (telemetry != null) {
            telemetry.addData("Turret Target Angle", "%.2f°", targetAngleDeg);
            telemetry.addData("Turret Current Angle", "%.2f°", servoToAngle(commandedServoPos));
            telemetry.addData("Turret Error", "%.2f°", lastError);
            telemetry.addData("Turret Servo Pos", "%.3f", commandedServoPos);
            telemetry.addData("Turret Aligned", isAligned());
            telemetry.addData("Turret Auto-Align", autoAlignEnabled);
            telemetry.addData("Distance to Goal", "%.2f\"", distanceToTarget);
        }
    }

    // ------------------ Public Control Methods ------------------

    /**
     * Enable automatic alignment to goal
     */
    public void enableAlignment() {
        autoAlignEnabled = true;
        // Reset PID state when re-enabling
        integral = 0;
        lastError = 0;
        firstRun = true;
    }

    /**
     * Disable automatic alignment (for manual control)
     */
    public void disableAlignment() {
        autoAlignEnabled = false;
    }

    /**
     * Check if auto-alignment is enabled
     */
    public boolean isAlignmentEnabled() {
        return autoAlignEnabled;
    }

    /**
     * Check if turret is aligned within tolerance
     */
    public boolean isAligned() {
        return Math.abs(lastError) < TOLERANCE;
    }

    /**
     * Manually set servo position (disables auto-alignment)
     */
    public void setManualPosition(double position) {
        autoAlignEnabled = false;
        position = clamp(position, SERVO_MIN, SERVO_MAX);
        commandedServoPos = position;
        turretServo1.setPosition(position);
        turretServo2.setPosition(position);
    }

    /**
     * Rotate turret left manually
     */
    public void rotateLeft(double amount) {
        autoAlignEnabled = false;
        double newPos = commandedServoPos - amount;
        newPos = clamp(newPos, SERVO_MIN, SERVO_MAX);
        commandedServoPos = newPos;
        turretServo1.setPosition(newPos);
        turretServo2.setPosition(newPos);
    }

    /**
     * Rotate turret right manually
     */
    public void rotateRight(double amount) {
        autoAlignEnabled = false;
        double newPos = commandedServoPos + amount;
        newPos = clamp(newPos, SERVO_MIN, SERVO_MAX);
        commandedServoPos = newPos;
        turretServo1.setPosition(newPos);
        turretServo2.setPosition(newPos);
    }

    /**
     * Center the turret
     */
    public void centerTurret() {
        commandedServoPos = SERVO_CENTER;
        turretServo1.setPosition(SERVO_CENTER);
        turretServo2.setPosition(SERVO_CENTER);
        integral = 0;
        lastError = 0;
    }

    /**
     * Stop turret at current position and disable auto-align
     */
    public void stopTurret() {
        autoAlignEnabled = false;
        // Servos hold position automatically
    }

    /**
     * Stop turret and re-enable auto-alignment
     */
    public void stopAndEnableAlign() {
        enableAlignment();
    }

    /**
     * Set telemetry for debugging
     */
    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    /**
     * Check if turret is ready to shoot
     */
    public boolean isReadyToShoot(double minDistance, double maxDistance) {
        return isAligned() &&
                distanceToTarget >= minDistance &&
                distanceToTarget <= maxDistance;
    }

    /**
     * Set target position on field
     */
    public void setTarget(double x, double y) {
        xt = x;
        yt = y;
    }

    // ------------------ Getters ------------------
    public double getX() { return x; }
    public double getY() { return y; }
    public double getHeading() { return heading; }
    public double getTargetAngleDeg() { return targetAngleDeg; }
    public double getTurretAngleDeg() { return servoToAngle(commandedServoPos); }
    public double getDistanceToTarget() { return distanceToTarget; }
    public double getLastError() { return lastError; }
    public double getIntegral() { return integral; }
    public double getCommandedServoPos() { return commandedServoPos; }
}