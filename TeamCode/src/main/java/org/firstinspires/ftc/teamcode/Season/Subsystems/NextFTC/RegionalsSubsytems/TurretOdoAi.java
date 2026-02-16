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

    // ------------------ Turret ------------------
    private double targetAngleDeg = 0;
    private double turretAngleDeg = 0;
    private double distanceToTarget = 0;

    // ========== ANGLE OFFSET (TUNE THIS TO FIX POINTING!) ==========
    // Positive values rotate the turret clockwise, negative counter-clockwise
    // Adjust in small increments (5-10 degrees) until turret points accurately at target
    public static double ANGLE_OFFSET = -25;  // degrees

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
    public static double MAX_VELOCITY = 1000;
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

    // ========== CRASH PREVENTION: Pose caching ==========
    private Pose cachedPose = null;
    private double lastPoseFetchTime = 0;
    private static final double POSE_CACHE_DURATION = 0.02;

    // ========== CRASH PREVENTION: Error tracking ==========
    private int consecutiveErrors = 0;
    private static final int MAX_CONSECUTIVE_ERRORS = 10;
    private boolean errorRecoveryMode = false;

    // ========== CRASH PREVENTION: Servo health monitoring ==========
    private double lastServo1Position = 0.25;
    private double lastServo2Position = 0.25;
    private int servoReadFailures = 0;
    private static final int MAX_SERVO_READ_FAILURES = 5;

    // ========== CRASH PREVENTION: Bounds checking ==========
    private static final double EMERGENCY_STOP_THRESHOLD = 0.95;

    private TurretOdoAi() {
    }

    // ------------------ Initialization ------------------
    public void init(HardwareMap hardwareMap) {
        try {
            turretServo1 = hardwareMap.get(Servo.class, "turretServo1");
            turretServo2 = hardwareMap.get(Servo.class, "turretServo2");

            if (!verifyServoHealth()) {
                throw new Exception("Servos not responding");
            }

            setSafeServoPosition(0.25);
            manualPosition = 0.25;
            lastServo1Position = 0.25;
            lastServo2Position = 0.25;

            loopTimer.reset();
            lastUpdateTime = loopTimer.seconds();
            firstRun = true;

            hardwareInitialized = true;
            consecutiveErrors = 0;
            errorRecoveryMode = false;

        } catch (Exception e) {
            hardwareInitialized = false;
            errorRecoveryMode = true;
        }
    }

    private boolean verifyServoHealth() {
        try {
            if (turretServo1 == null || turretServo2 == null) {
                return false;
            }

            double pos1 = turretServo1.getPosition();
            double pos2 = turretServo2.getPosition();

            return (pos1 >= 0.0 && pos1 <= 1.0) && (pos2 >= 0.0 && pos2 <= 1.0);

        } catch (Exception e) {
            return false;
        }
    }

    private void setSafeServoPosition(double position) {
        try {
            position = clamp(position, SERVO_MIN, SERVO_MAX);

            if (turretServo1 != null) {
                turretServo1.setPosition(position);
                lastServo1Position = position;
            }

            if (turretServo2 != null) {
                turretServo2.setPosition(position);
                lastServo2Position = position;
            }

            servoReadFailures = 0;

        } catch (Exception e) {
            servoReadFailures++;
            if (servoReadFailures >= MAX_SERVO_READ_FAILURES) {
                hardwareInitialized = false;
                errorRecoveryMode = true;
            }
        }
    }

    private double getSafeServoPosition() {
        try {
            if (turretServo1 == null) {
                return lastServo1Position;
            }

            double position = turretServo1.getPosition();

            if (position < 0.0 || position > 1.0) {
                return lastServo1Position;
            }

            lastServo1Position = position;
            servoReadFailures = 0;
            return position;

        } catch (Exception e) {
            servoReadFailures++;
            if (servoReadFailures >= MAX_SERVO_READ_FAILURES) {
                hardwareInitialized = false;
                errorRecoveryMode = true;
            }
            return lastServo1Position;
        }
    }

    // ------------------ Manual Control ------------------

    public void incrementPosition(double delta) {
        if (!hardwareInitialized || errorRecoveryMode) return;

        try {
            manualPosition = clamp(manualPosition + delta, SERVO_MIN, SERVO_MAX);

            if (manualPosition >= EMERGENCY_STOP_THRESHOLD || manualPosition <= (1.0 - EMERGENCY_STOP_THRESHOLD)) {
                manualPosition = clamp(manualPosition, 0.05, 0.95);
            }

            if (Math.abs(getSafeServoPosition() - manualPosition) > 0.002) {
                setSafeServoPosition(manualPosition);
            }

        } catch (Exception e) {
            consecutiveErrors++;
            if (consecutiveErrors >= MAX_CONSECUTIVE_ERRORS) {
                errorRecoveryMode = true;
            }
        }
    }

    public void turnRight() {
        incrementPosition(0.05);
    }

    public void turnLeft() {
        incrementPosition(-0.05);
    }

    public void continuousTurnRight(double speed) {
        if (!hardwareInitialized || errorRecoveryMode) return;

        try {
            double delta = 0.01 * speed;
            manualPosition = clamp(manualPosition + delta, SERVO_MIN, SERVO_MAX);

            if (manualPosition >= EMERGENCY_STOP_THRESHOLD) {
                manualPosition = EMERGENCY_STOP_THRESHOLD;
                return;
            }

            setSafeServoPosition(manualPosition);

        } catch (Exception e) {
            consecutiveErrors++;
            if (consecutiveErrors >= MAX_CONSECUTIVE_ERRORS) {
                errorRecoveryMode = true;
            }
        }
    }

    public void continuousTurnLeft(double speed) {
        if (!hardwareInitialized || errorRecoveryMode) return;

        try {
            double delta = -0.01 * speed;
            manualPosition = clamp(manualPosition + delta, SERVO_MIN, SERVO_MAX);

            if (manualPosition <= (1.0 - EMERGENCY_STOP_THRESHOLD)) {
                manualPosition = (1.0 - EMERGENCY_STOP_THRESHOLD);
                return;
            }

            setSafeServoPosition(manualPosition);

        } catch (Exception e) {
            consecutiveErrors++;
            if (consecutiveErrors >= MAX_CONSECUTIVE_ERRORS) {
                errorRecoveryMode = true;
            }
        }
    }

    public void setAutoMode() {
        manualMode = false;

        integral = 0;
        lastError = 0;
        firstRun = true;
        lastUpdateTime = loopTimer.seconds();

        cachedPose = null;

        consecutiveErrors = 0;

        if (errorRecoveryMode && verifyServoHealth()) {
            errorRecoveryMode = false;
            hardwareInitialized = true;
        }
    }

    public void setManualMode() {
        manualMode = true;

        if (hardwareInitialized && turretServo1 != null) {
            try {
                manualPosition = getSafeServoPosition();
            } catch (Exception e) {
                manualPosition = 0.25;
            }
        }

        consecutiveErrors = 0;

        if (errorRecoveryMode && verifyServoHealth()) {
            errorRecoveryMode = false;
            hardwareInitialized = true;
        }
    }

    private Pose getCachedPose() {
        try {
            double currentTime = loopTimer.seconds();

            if (cachedPose != null && (currentTime - lastPoseFetchTime) < POSE_CACHE_DURATION) {
                return cachedPose;
            }

            if (PedroComponent.follower() == null) {
                return cachedPose;
            }

            Pose newPose = PedroComponent.follower().getPose();

            if (newPose != null) {
                cachedPose = newPose;
                lastPoseFetchTime = currentTime;
            }

            return cachedPose;

        } catch (Exception e) {
            return cachedPose;
        }
    }

    // ------------------ Loop ------------------
    @Override
    public void periodic() {
        if (errorRecoveryMode) {
            if (loopTimer.seconds() % 3.0 < 0.1) {
                if (verifyServoHealth()) {
                    errorRecoveryMode = false;
                    hardwareInitialized = true;
                    consecutiveErrors = 0;
                }
            }
            return;
        }

        // === 0. MANUAL MODE CHECK ===
        if (manualMode) {
            try {
                Pose currentPose = getCachedPose();
                if (currentPose != null) {
                    x = currentPose.getX() - 72;
                    y = currentPose.getY() - 72;
                    heading = Math.toDegrees(currentPose.getHeading());
                    heading = (heading + 360) % 360;

                    double fieldAngleDeg = Math.toDegrees(Math.atan2(yt - y, xt - x));
                    fieldAngleDeg = (fieldAngleDeg + 360) % 360;
                    distanceToTarget = Math.hypot(xt - x, yt - y);

                    // Apply angle offset in manual mode too (for telemetry)
                    targetAngleDeg = fieldAngleDeg - heading + 180 + ANGLE_OFFSET;
                    targetAngleDeg = normalizeDegrees(targetAngleDeg);

                    if (hardwareInitialized) {
                        currentServoPos = getSafeServoPosition();
                        turretAngleDeg = servoToAngle(currentServoPos);
                    }
                }
            } catch (Exception e) {
                consecutiveErrors++;
                if (consecutiveErrors >= MAX_CONSECUTIVE_ERRORS) {
                    errorRecoveryMode = true;
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
        if (!hardwareInitialized) {
            return;
        }

        try {
            // === 3. GET POSE ===
            Pose currentPose = getCachedPose();
            if (currentPose == null) {
                return;
            }

            // === 4. UPDATE ROBOT POSE ===
            x = currentPose.getX() - 72;
            y = currentPose.getY() - 72;
            heading = Math.toDegrees(currentPose.getHeading());
            heading = (heading + 360) % 360;

            // === 5. CALCULATE TARGET ANGLE WITH OFFSET ===
            double fieldAngleDeg = Math.toDegrees(Math.atan2(yt - y, xt - x));
            fieldAngleDeg = (fieldAngleDeg + 360) % 360;

            distanceToTarget = Math.hypot(xt - x, yt - y);

            // Apply base 180° offset + tunable ANGLE_OFFSET
            targetAngleDeg = fieldAngleDeg - heading + 180 + ANGLE_OFFSET;
            targetAngleDeg = normalizeDegrees(targetAngleDeg);

            // === 6. READ CURRENT TURRET POSITION ===
            currentServoPos = getSafeServoPosition();
            turretAngleDeg = servoToAngle(currentServoPos);

            // === 7. CALCULATE ERROR ===
            double error = targetAngleDeg - turretAngleDeg;

            if (error > 180) error -= 360;
            if (error < -180) error += 360;

            if (Math.abs(error) < 0.5) {
                lastUpdateTime = currentTime;
                consecutiveErrors = 0;
                return;
            }

            // === 8. CALCULATE TIME DELTA ===
            double dt = timeSinceLastUpdate;

            if (firstRun) {
                dt = MIN_LOOP_TIME;
                firstRun = false;
            }

            if (dt <= 0 || dt > 0.5) {
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
            newAngle = normalizeDegrees(newAngle);

            double newServoPos = angleToServo(newAngle);
            newServoPos = clamp(newServoPos, SERVO_MIN, SERVO_MAX);

            if (newServoPos >= EMERGENCY_STOP_THRESHOLD || newServoPos <= (1.0 - EMERGENCY_STOP_THRESHOLD)) {
                newServoPos = clamp(newServoPos, 0.05, 0.95);
            }

            if (Math.abs(newServoPos - currentServoPos) > 0.002) {
                setSafeServoPosition(newServoPos);
            }

            // === 12. UPDATE STATE ===
            lastError = error;
            lastUpdateTime = currentTime;
            consecutiveErrors = 0;

        } catch (Exception e) {
            consecutiveErrors++;
            if (consecutiveErrors >= MAX_CONSECUTIVE_ERRORS) {
                errorRecoveryMode = true;
                hardwareInitialized = false;
            }
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

    public double normalizeDegrees(double angle) {
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
    public double getIntegral() { return integral; }
    public double getKp() { return kP; }
    public int getSkippedLoops() { return skippedLoops; }
    public double getLoopTime() { return loopTimer.seconds() - lastUpdateTime; }
    public boolean isManualMode() { return manualMode; }
    public double getManualPosition() { return manualPosition; }
    public double getAngleOffset() { return ANGLE_OFFSET; }

    public boolean isErrorRecoveryMode() { return errorRecoveryMode; }
    public int getConsecutiveErrors() { return consecutiveErrors; }
}
