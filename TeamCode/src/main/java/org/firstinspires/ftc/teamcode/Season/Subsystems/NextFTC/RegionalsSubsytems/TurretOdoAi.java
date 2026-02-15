package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;

public class TurretOdoAi implements Subsystem {

    public static final TurretOdoAi INSTANCE = new TurretOdoAi();

    // ------------------ Hardware ------------------
    private Servo turretServo1;
    private Servo turretServo2;

    // ------------------ Robot Pose ------------------
    private double x = 0;
    private double y = 0;
    private double heading = 0;

    // ------------------ Target (Red Goal) ------------------
    public static double xt = 58;
    public static double yt = 58;

    // ------------------ Turret ------------------
    private double targetAngleDeg = 0;      // Where turret should point
    private double turretAngleDeg = 0;      // Where turret currently is
    private double distanceToTarget = 0;

    // Servo safety
    public static double SERVO_MIN = 0.0;
    public static double SERVO_MAX = 1.0;
    public boolean hardwareInitialized = false;

    // ========== PID CONSTANTS (TUNE THESE!) ==========
    public static double kP = 0.100;         // Proportional gain
    public static double kI = 0.0;          // Integral gain (start at 0)
    public static double kD = 0.003;        // Derivative gain

    // Motion limits
    public static double MAX_VELOCITY = 500.0;  // deg/sec max speed
    public static double TOLERANCE = 2.0;       // degrees (acceptable error)

    // ========== PID STATE VARIABLES ==========
    private double lastError = 0;
    private double integral = 0;
    private double lastTime = 0;
    private boolean firstRun = true;

    private TurretOdoAi() {
    }

    // ------------------ Initialization ------------------
    public void init(HardwareMap hardwareMap) {
        turretServo1 = hardwareMap.get(Servo.class, "turretServo1");
        turretServo2 = hardwareMap.get(Servo.class, "turretServo2");
        turretServo1.setPosition(0.25);
        turretServo2.setPosition(0.25);

        // Initialize time tracking
        lastTime = System.nanoTime() / 1e9;
        firstRun = true;

        hardwareInitialized = true;
    }

    // ------------------ Loop ------------------
    @Override
    public void periodic() {
        // === 0. HARDWARE CHECK ===
        if (!hardwareInitialized || turretServo1 == null || turretServo2 == null) {
            return; // Skip if hardware not initialized
        }

        try {
            // === 1. SAFETY CHECKS ===
            if (PedroComponent.follower() == null) {
                return;
            }

            // DO NOT CALL follower.update() - it's called in main loop

            Pose currentPose = PedroComponent.follower().getPose();
            if (currentPose == null) {
                return;
            }

            // === 2. UPDATE ROBOT POSE ===
            x = -currentPose.getX();
            y = -currentPose.getY();
            heading = Math.toDegrees(currentPose.getHeading());
            heading = (heading + 360) % 360;
            // === 3. CALCULATE TARGET ANGLE ===
            // Field-centric angle to goal
            double fieldAngleDeg = Math.toDegrees(Math.atan2(yt - y, xt - x));
            fieldAngleDeg = (fieldAngleDeg + 360) % 360;

            // Distance to target
            distanceToTarget = Math.hypot(xt - x, yt - y);

            // Robot-centric target angle (where turret should point)
            targetAngleDeg = fieldAngleDeg - heading;
            targetAngleDeg = normalizeDegrees(targetAngleDeg);

            // === 4. DIRECT SERVO CONTROL (NO PID) ===
            // Convert target angle directly to servo position
            double servoPos = angleToServo(targetAngleDeg);
            servoPos = clamp(servoPos, SERVO_MIN, SERVO_MAX);

            // Set servos to target position immediately
            turretServo1.setPosition(servoPos);
            turretServo2.setPosition(servoPos);

            // Update current turret angle for telemetry
            turretAngleDeg = targetAngleDeg;

            // Calculate error for telemetry
            lastError = 0; // No error since we're setting directly to target

        } catch (Exception e) {
            // Catch any errors in periodic to prevent crashes
        }

//    public void periodic() {
//        // === 1. SAFETY CHECKS ===
//        if (PedroComponent.follower() == null) {
//            return;
//        }
//
//        PedroComponent.follower().update();
//
//        Pose currentPose = PedroComponent.follower().getPose();
//        if (currentPose == null) {
//            return;
//        }
//
//        // === 2. UPDATE ROBOT POSE ===
//        x = currentPose.getX() - 72;
//        y = currentPose.getY() - 72;
//        heading = Math.toDegrees(currentPose.getHeading());
//        heading = (heading + 360) % 360;
//
//        // === 3. CALCULATE TARGET ANGLE ===
//        // Field-centric angle to goal
//        double fieldAngleDeg = Math.toDegrees(Math.atan2(yt - y, xt - x));
//        fieldAngleDeg = (fieldAngleDeg + 360) % 360;
//
//        // Distance to target
//        distanceToTarget = Math.hypot(xt - x, yt - y);
//
//        // Robot-centric target angle (where turret should point)
//        targetAngleDeg = fieldAngleDeg - heading;
//        targetAngleDeg = normalizeDegrees(targetAngleDeg);
//
//        // === 4. READ CURRENT TURRET POSITION ===
//        // Get servo position and convert to angle
//        double currentServoPos = turretServo1.getPosition();
//        turretAngleDeg = servoToAngle(currentServoPos);
//
//        // === 5. CALCULATE ERROR (SHORTEST PATH) ===
//        double error = targetAngleDeg - turretAngleDeg;
//
//        // Wrap error to -180 to +180 (shortest rotation path)
//        if (error > 180) error -= 360;
//        if (error < -180) error += 360;
//
//        // === 6. CALCULATE TIME DELTA ===
//        double currentTime = System.nanoTime() / 1e9;  // Convert to seconds
//        double dt = currentTime - lastTime;
//
//        if (firstRun) {
//            dt = 0.02;  // Assume 20ms on first run
//            firstRun = false;
//        }
//
//        if (dt <= 0 || dt > 0.1) {
//            dt = 0.02;  // Safety: default to 20ms if time is weird
//        }
//
//        // === 7. PID CALCULATIONS ===
//
//        // P term: Proportional to error
//        double P_output = kP * error;
//
//        // I term: Integral (accumulated error over time)
//        integral += error * dt;
//
//        // Anti-windup: Reset integral if we're close to target
//        if (Math.abs(error) < TOLERANCE) {
//            integral = 0;
//        }
//
//        // Clamp integral to prevent runaway accumulation
//        integral = clamp(integral, -100, 100);
//        double I_output = kI * integral;
//
//        // D term: Derivative (rate of change of error)
//        double derivative = (error - lastError) / dt;
//        double D_output = kD * derivative;
//
//        // === 8. COMBINE PID OUTPUTS ===
//        double pidOutput = P_output + I_output + D_output;
//
//        // Limit to max velocity (deg/sec)
//        pidOutput = clamp(pidOutput, -MAX_VELOCITY, MAX_VELOCITY);
//
//        // === 9. UPDATE SERVO POSITION ===
//        // Convert velocity to position change
//        double positionChange = pidOutput * dt;
//        double newAngle = turretAngleDeg + positionChange;
//        newAngle = normalizeDegrees(newAngle);
//
//        // Convert angle to servo position and apply
//        double newServoPos = angleToServo(newAngle);
//        newServoPos = clamp(newServoPos, SERVO_MIN, SERVO_MAX);
//
//        turretServo1.setPosition(newServoPos);
//        turretServo2.setPosition(newServoPos);  // Mirror servo if needed
//
//        // === 10. UPDATE STATE FOR NEXT LOOP ===
//        lastError = error;
//        lastTime = currentTime;
    }

    // ------------------ Helper Functions ------------------

    /**
     * Convert angle in degrees to servo position (0.0 to 1.0)
     * Includes your custom mapping with inversion
     */
    private double angleToServo(double angleDeg) {
        angleDeg = normalizeDegrees(angleDeg);
        double pos = 1.0 - ((angleDeg + 180) / 360.0);
        return clamp(pos, SERVO_MIN, SERVO_MAX);
    }

    /**
     * Convert servo position (0.0 to 1.0) back to angle in degrees
     * Reverse of angleToServo()
     */
    private double servoToAngle(double servoPos) {
        // Reverse the formula: pos = 1 - (angle + 180)/360
        // 1 - pos = (angle + 180)/360
        // 360 * (1 - pos) = angle + 180
        // angle = 360 * (1 - pos) - 180
        double angle = 360.0 * (1.0 - servoPos) - 180.0;
        return normalizeDegrees(angle);
    }

    /**
     * Normalize angle to -180 to +180 range
     */
    private double normalizeDegrees(double angle) {
        angle = (angle + 360) % 360;
        if (angle > 180) angle -= 360;
        return angle;
    }

    /**
     * Clamp value between min and max
     */
    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    // ------------------ Getters ------------------
    public double getX() { return x; }
    public double getY() { return y; }
    public double getHeading() { return heading; }
    public double getTargetAngleDeg() { return targetAngleDeg; }
    public double getTurretAngleDeg() { return turretAngleDeg; }
    public double getDistanceToTarget() { return distanceToTarget; }
    public double getLastError() { return lastError; }  // For debugging
    public double getIntegral() { return integral; }    // For debugging
}