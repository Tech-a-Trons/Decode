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
    private boolean hardwareInitialized = false;

    // ------------------ Robot Pose ------------------
    private double x = 0;
    private double y = 0;
    private double heading = 0;

    // ------------------ Target (Red Goal) ------------------
    public static double xt = 121;
    public static double yt = 121;

    // ------------------ Turret ------------------
    private double targetAngleDeg = 0;      // Where turret should point
    private double turretAngleDeg = 0;      // Where turret currently is
    private double distanceToTarget = 0;

    // Servo safety
    public static double SERVO_MIN = 0; //0
    public static double SERVO_MAX = 1.0;

    // ------------------ ANGLE OFFSET ------------------
    // Adjust this value to compensate for real-world mounting/calibration differences
    // Positive values rotate the turret clockwise, negative counter-clockwise
    public static double ANGLE_OFFSET_DEG = 290;

    private double lastError = 0;

    private TurretOdoAi() {
    }

    // ------------------ Initialization ------------------
    public void init(HardwareMap hardwareMap) {
        try {
            turretServo1 = hardwareMap.get(Servo.class, "turretServo1");
            turretServo2 = hardwareMap.get(Servo.class, "turretServo2");

            if (turretServo1 != null) {
                turretServo1.setPosition(0.5);
            }
            if (turretServo2 != null) {
                turretServo2.setPosition(0.5);
            }

            hardwareInitialized = true;

        } catch (Exception e) {
            hardwareInitialized = false;
            // Silently fail - error will be logged in telemetry
        }
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
            x = currentPose.getX();
            y = currentPose.getY();
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

            // === 4. APPLY ANGLE OFFSET ===
            // Add the offset to compensate for real-world mounting differences
            double adjustedAngleDeg = targetAngleDeg + ANGLE_OFFSET_DEG;
            adjustedAngleDeg = normalizeDegrees(adjustedAngleDeg);

            // === 5. DIRECT SERVO CONTROL (NO PID) ===
            // Convert adjusted angle to servo position
            double servoPos = angleToServo(adjustedAngleDeg);
            servoPos = clamp(servoPos, SERVO_MIN, SERVO_MAX);

            // Set servos to target position immediately
            turretServo1.setPosition(servoPos);
            turretServo2.setPosition(servoPos);

            // Update current turret angle for telemetry (with offset applied)
            turretAngleDeg = adjustedAngleDeg;

            // Calculate error for telemetry
            lastError = 0; // No error since we're setting directly to target

        } catch (Exception e) {
            // Catch any errors in periodic to prevent crashes
        }
    }

    // ------------------ Helper Functions ------------------

    public boolean turretoff() {
        return false;
    }

    private double angleToServo(double angleDeg) {
        angleDeg = normalizeDegrees(angleDeg);
        double pos = 1.0 - ((angleDeg + 180) / 360.0);
        return clamp(pos, SERVO_MIN, SERVO_MAX);
    }

    private double servoToAngle(double servoPos) {
        double angle = 360.0 * (1.0 - servoPos) - 180.0;
        return normalizeDegrees(angle);
    }

    private double normalizeDegrees(double angle) {
        angle = (angle + 360) % 360;
        if (angle > 180) angle -= 360;
        return angle;
    }

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
    public double getLastError() { return lastError; }
    public boolean isHardwareInitialized() { return hardwareInitialized; }
    public double getAngleOffset() { return ANGLE_OFFSET_DEG; }
}