package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    public static double xt = 68;
    public static double yt = 68;

    double AngleOffset = -25;

    // ------------------ Turret ------------------
    private double targetAngleDeg = 0;
    private double turretAngleDeg = 0;
    private double distanceToTarget = 0;

    // Servo safety
    public static double SERVO_MIN = 0.0;
    public static double SERVO_MAX = 1.0;
    public boolean hardwareInitialized = false;

    // Manual mode
    public boolean manualMode = false;
    private double manualPosition = 0.25;

    // ========== PID CONSTANTS ==========
    public static double kP = 9.00;
    public static double kI = 0.000;
    public static double kD = 0.012;

    double currentServoPos = 0;

    // Motion limits
    public static double MAX_VELOCITY = 800.0;
    public static double TOLERANCE = 2.0;

    // ========== PID STATE VARIABLES ==========
    private double lastError = 0;
    private double integral = 0;
    private double lastUpdateTime = 0;
    private boolean firstRun = true;

    // ========== RATE LIMITING ==========
    private ElapsedTime loopTimer = new ElapsedTime();
    private static final double MIN_LOOP_TIME = 0.015;
    private int skippedLoops = 0;

    private TurretOdoAi() {
    }

    // ------------------ Initialization ------------------
    public void init(HardwareMap hardwareMap) {
        try {
            turretServo1 = hardwareMap.get(Servo.class, "turretServo1");
            turretServo2 = hardwareMap.get(Servo.class, "turretServo2");

            // Set safe initial position
            turretServo1.setPosition(0.25);
            turretServo2.setPosition(0.25);
            manualPosition = 0.25;

            // Initialize timers
            loopTimer.reset();
            lastUpdateTime = loopTimer.seconds();
            firstRun = true;

            hardwareInitialized = true;

        } catch (Exception e) {
            hardwareInitialized = false;
        }
    }

    // ------------------ Manual Control ------------------

    public void incrementPosition(double delta) {
        if (!hardwareInitialized) return;

        manualPosition = clamp(manualPosition + delta, SERVO_MIN, SERVO_MAX);

        if (Math.abs(turretServo1.getPosition() - manualPosition) > 0.002) {
            turretServo1.setPosition(manualPosition);
            turretServo2.setPosition(manualPosition);
        }
    }

    public void turnRight() {
        incrementPosition(0.05);
    }

    public void turnLeft() {
        incrementPosition(-0.05);
    }

    public void continuousTurnRight(double speed) {
        if (!hardwareInitialized) return;

        double delta = 0.01 * speed;
        manualPosition = clamp(manualPosition + delta, SERVO_MIN, SERVO_MAX);

        turretServo1.setPosition(manualPosition);
        turretServo2.setPosition(manualPosition);
    }

    public void continuousTurnLeft(double speed) {
        if (!hardwareInitialized) return;

        double delta = -0.01 * speed;
        manualPosition = clamp(manualPosition + delta, SERVO_MIN, SERVO_MAX);

        turretServo1.setPosition(manualPosition);
        turretServo2.setPosition(manualPosition);
    }

    public void setAutoMode() {
        manualMode = false;

        // Reset PID state when entering auto mode
        integral = 0;
        lastError = 0;
        firstRun = true;
        lastUpdateTime = loopTimer.seconds();
    }

    public void setManualMode() {
        manualMode = true;

        // Sync manual position with current servo position
        if (hardwareInitialized && turretServo1 != null) {
            manualPosition = turretServo1.getPosition();
        }
    }

    // ------------------ Loop ------------------
    @Override
    public void periodic() {
        // === 0. MANUAL MODE CHECK ===
        if (manualMode) {
            if (PedroComponent.follower() != null) {
                Pose currentPose = PedroComponent.follower().getPose();
                if (currentPose != null) {
                    x = currentPose.getX() - 72;
                    y = currentPose.getY() - 72;
                    heading = Math.toDegrees(currentPose.getHeading());
                    heading = (heading + 360) % 360;

                    double fieldAngleDeg = Math.toDegrees(Math.atan2(yt - y, xt - x));
                    fieldAngleDeg = (fieldAngleDeg + 360) % 360;
                    distanceToTarget = Math.hypot(xt - x, yt - y);
                    targetAngleDeg = fieldAngleDeg - heading + 180 + AngleOffset;
                    targetAngleDeg = normalizeDegrees(targetAngleDeg);

                    if (hardwareInitialized && turretServo1 != null) {
                        currentServoPos = turretServo1.getPosition();
                        turretAngleDeg = servoToAngle(currentServoPos);
                    }
                }
            }
            return;
        }

        // === 1. RATE LIMITING ===
        double currentTime = loopTimer.seconds();
        double timeSinceLastUpdate = currentTime - lastUpdateTime;

        if (timeSinceLastUpdate < MIN_LOOP_TIME) {
            skippedLoops++;
            return;
        }

        // === 2. HARDWARE CHECK ===
        if (!hardwareInitialized || turretServo1 == null || turretServo2 == null) {
            return;
        }

        try {
            // === 3. GET POSE ===
            if (PedroComponent.follower() == null) {
                return;
            }

            Pose currentPose = PedroComponent.follower().getPose();
            if (currentPose == null) {
                return;
            }

            // === 4. UPDATE ROBOT POSE ===
            x = currentPose.getX() - 72;
            y = currentPose.getY() - 72;
            heading = Math.toDegrees(currentPose.getHeading());
            heading = (heading + 360) % 360;

            // === 5. CALCULATE TARGET ANGLE ===
            double fieldAngleDeg = Math.toDegrees(Math.atan2(yt - y, xt - x));
            fieldAngleDeg = (fieldAngleDeg + 360) % 360;

            distanceToTarget = Math.hypot(xt - x, yt - y);

            targetAngleDeg = fieldAngleDeg - heading + 180 + AngleOffset;
            targetAngleDeg = normalizeDegrees(targetAngleDeg);

            // === 6. READ CURRENT TURRET POSITION ===
            currentServoPos = turretServo1.getPosition();
            turretAngleDeg = servoToAngle(currentServoPos);

            // ========== WRAPPING FIX: Calculate shortest path error ==========
            // This is the key: always take the SHORTEST angular distance
            double error = targetAngleDeg - turretAngleDeg;

            // Wrap error to shortest path (-180 to +180)
            while (error > 180) error -= 360;
            while (error < -180) error += 360;

            // Now error is guaranteed to be the shortest rotation direction
            // Examples:
            // Target: 170°, Turret: -170° → Error: 340° → Wrapped: -20° (turn left 20°)
            // Target: -170°, Turret: 170° → Error: -340° → Wrapped: 20° (turn right 20°)

            // Skip updates if error is very small
            if (Math.abs(error) < 0.5) {
                lastUpdateTime = currentTime;
                return;
            }

            // === 8. CALCULATE TIME DELTA ===
            double dt = timeSinceLastUpdate;

            if (firstRun) {
                dt = MIN_LOOP_TIME;
                firstRun = false;
            }

            if (dt <= 0 || dt > 0.2) {
                dt = MIN_LOOP_TIME;
            }

            // === 9. PID CALCULATIONS ===
            double P_output = kP * error;

            integral += error * dt;

            if (Math.abs(error) < TOLERANCE) {
                integral = 0;
            }

            integral = clamp(integral, -100, 100);
            double I_output = kI * integral;

            double derivative = (error - lastError) / dt;
            double D_output = kD * derivative;

            // === 10. COMBINE PID OUTPUTS ===
            double pidOutput = P_output + I_output + D_output;
            pidOutput = clamp(pidOutput, -MAX_VELOCITY, MAX_VELOCITY);

            // === 11. UPDATE SERVO POSITION ===
            double positionChange = pidOutput * dt;
            double newAngle = turretAngleDeg + positionChange;

            // Normalize the new angle
            newAngle = normalizeDegrees(newAngle);

            double newServoPos = angleToServo(newAngle);
            newServoPos = clamp(newServoPos, SERVO_MIN, SERVO_MAX);

            // Only update servos if position changed significantly
            if (Math.abs(newServoPos - currentServoPos) > 0.002) {
                turretServo1.setPosition(newServoPos);
                turretServo2.setPosition(newServoPos);
            }

            // === 12. UPDATE STATE ===
            lastError = error;
            lastUpdateTime = currentTime;

        } catch (Exception e) {
            // Silent catch to prevent crashes
        }
    }

    // ------------------ Helper Functions ------------------

    private double angleToServo(double angleDeg) {
        angleDeg = normalizeDegrees(angleDeg);
        double pos = 1.0 - ((angleDeg + 180) / 360.0);
        return clamp(pos, SERVO_MIN, SERVO_MAX);
    }

    private double servoToAngle(double servoPos) {
        double angle = 360.0 * (1.0 - servoPos) - 180.0;
        return normalizeDegrees(angle);
    }

    /**
     * ========== WRAPPING FIX: Simple and robust normalization ==========
     * Always returns angle in range [-180, +180]
     */
    public double normalizeDegrees(double angle) {
        // Use while loops to handle any magnitude
        while (angle > 180) {
            angle -= 360;
        }
        while (angle < -180) {
            angle += 360;
        }
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
    public double getIntegral() { return integral; }
    public double getKp() { return kP; }
    public int getSkippedLoops() { return skippedLoops; }
    public double getLoopTime() { return loopTimer.seconds() - lastUpdateTime; }
    public boolean isManualMode() { return manualMode; }
    public double getManualPosition() { return manualPosition; }
}