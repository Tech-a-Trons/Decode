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
    public static double xt = 121;
    public static double yt = 121;

    double AngleOffset =   -25-96;


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
    public static double kP = 40.00;
    public static double kI = 0.001;
    public static double kD = 0.04;

    double currentServoPos = 0;

    // Motion limits
    public static double MAX_VELOCITY = 800.0;
    public static double TOLERANCE = 0.0;

    // ========== PID STATE VARIABLES ==========
    private double lastError = 0;
    private double integral = 0;
    private double lastUpdateTime = 0;
    private boolean firstRun = true;

    // ========== RATE LIMITING ==========
    private ElapsedTime loopTimer = new ElapsedTime();
    private static final double MIN_LOOP_TIME = 0.010;  // Reduced to 10ms for faster updates
    private int skippedLoops = 0;

    // ========== IMPROVED WRAPPING FIX ==========
    // Track the actual commanded position (before servo limits)
    private double commandedAngle = 0;
    private boolean commandedAngleInitialized = false;

    // ========== PERFORMANCE OPTIMIZATIONS ==========
    private Pose cachedPose = null;  // Cache pose reference
    private static final double DEG_TO_RAD = Math.PI / 180.0;
    private static final double RAD_TO_DEG = 180.0 / Math.PI;

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

            // Initialize commanded angle tracking
            commandedAngle = servoToAngle(0.25);
            commandedAngleInitialized = true;

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

            // Update commanded angle when manually moving
            commandedAngle = servoToAngle(manualPosition);
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

        // Update commanded angle
        commandedAngle = servoToAngle(manualPosition);
    }

    public void continuousTurnLeft(double speed) {
        if (!hardwareInitialized) return;

        double delta = -0.01 * speed;
        manualPosition = clamp(manualPosition + delta, SERVO_MIN, SERVO_MAX);

        turretServo1.setPosition(manualPosition);
        turretServo2.setPosition(manualPosition);

        // Update commanded angle
        commandedAngle = servoToAngle(manualPosition);
    }

    public void setAutoMode() {
        manualMode = false;

        // Reset PID state when entering auto mode
        integral = 0;
        lastError = 0;
        firstRun = true;
        lastUpdateTime = loopTimer.seconds();

        // Initialize commanded angle from current position
        if (hardwareInitialized && turretServo1 != null) {
            currentServoPos = turretServo1.getPosition();
            commandedAngle = servoToAngle(currentServoPos);
            commandedAngleInitialized = true;
        }
    }

    public void setManualMode() {
        manualMode = true;

        // Sync manual position with current servo position
        if (hardwareInitialized && turretServo1 != null) {
            manualPosition = turretServo1.getPosition();
            commandedAngle = servoToAngle(manualPosition);
        }
    }

    // ------------------ Loop ------------------
    @Override
    public void periodic() {
        // === EARLY EXIT: Manual mode lightweight update ===
        if (manualMode) {
            updateManualModeTelemetry();
            return;
        }

        // === EARLY EXIT: Hardware check ===
        if (!hardwareInitialized) return;

        // === EARLY EXIT: Rate limiting ===
        double currentTime = loopTimer.seconds();
        double timeSinceLastUpdate = currentTime - lastUpdateTime;

        if (timeSinceLastUpdate < MIN_LOOP_TIME) {
            skippedLoops++;
            return;
        }

        // === OPTIMIZED: Single follower null check ===
        if (PedroComponent.follower() == null) return;

        try {
            // === OPTIMIZED: Get pose once and cache ===
            cachedPose = PedroComponent.follower().getPose();
            if (cachedPose == null) return;

            // === OPTIMIZED: Direct field access instead of getters ===
            x = cachedPose.getX() - 72;
            y = cachedPose.getY() - 72;

            // === OPTIMIZED: Use radians directly, convert once ===
            double headingRad = cachedPose.getHeading();
            heading = Math.toDegrees(headingRad);
            if (heading < 0) heading += 360;

            // === OPTIMIZED: Combined angle calculation ===
            double dx = xt - x;
            double dy = yt - y;
            distanceToTarget = Math.sqrt(dx * dx + dy * dy);  // Slightly faster than hypot

            double fieldAngleDeg = Math.toDegrees(Math.atan2(dy, dx));
            if (fieldAngleDeg < 0) fieldAngleDeg += 360;

            targetAngleDeg = fieldAngleDeg - heading + 180 + AngleOffset;
            targetAngleDeg = normalizeDegrees(targetAngleDeg);

            // === READ CURRENT POSITION ===
            currentServoPos = turretServo1.getPosition();
            turretAngleDeg = servoToAngle(currentServoPos);

            // === Initialize commanded angle if needed ===
            if (!commandedAngleInitialized) {
                commandedAngle = turretAngleDeg;
                commandedAngleInitialized = true;
            }

            // === CALCULATE ERROR ===
            double error = targetAngleDeg - commandedAngle;

            // Wrap error to shortest path
            if (error > 180) error -= 360;
            else if (error < -180) error += 360;

            // === EARLY EXIT: Skip if within tolerance ===
            if (Math.abs(error) < 0.5) {
                lastUpdateTime = currentTime;
                return;
            }

            // === CALCULATE TIME DELTA ===
            double dt = firstRun ? MIN_LOOP_TIME : timeSinceLastUpdate;
            if (dt <= 0 || dt > 0.2) dt = MIN_LOOP_TIME;
            firstRun = false;

            // === PID CALCULATION ===
            double P_output = kP * error;

            integral += error * dt;
            if (Math.abs(error) < TOLERANCE) integral = 0;
            integral = clamp(integral, -100, 100);
            double I_output = kI * integral;

            double derivative = (error - lastError) / dt;
            double D_output = kD * derivative;

            double pidOutput = clamp(P_output + I_output + D_output, -MAX_VELOCITY, MAX_VELOCITY);

            // === UPDATE POSITION ===
            commandedAngle += pidOutput * dt;
            double normalizedAngle = normalizeDegrees(commandedAngle);
            double newServoPos = angleToServo(normalizedAngle);
            newServoPos = clamp(newServoPos, SERVO_MIN, SERVO_MAX);

            // === OPTIMIZED: Single comparison, batch servo writes ===
            if (Math.abs(newServoPos - currentServoPos) > 0.002) {
                turretServo1.setPosition(newServoPos);
                turretServo2.setPosition(newServoPos);
            }

            // === UPDATE STATE ===
            lastError = error;
            lastUpdateTime = currentTime;

        } catch (Exception e) {
            // Silent catch to prevent crashes
        }
    }

    // === OPTIMIZED: Lightweight manual mode telemetry update ===
    private void updateManualModeTelemetry() {
        if (PedroComponent.follower() != null) {
            Pose currentPose = PedroComponent.follower().getPose();
            if (currentPose != null) {
                x = currentPose.getX() - 72;
                y = currentPose.getY() - 72;
                heading = Math.toDegrees(currentPose.getHeading());
                if (heading < 0) heading += 360;

                double dx = xt - x;
                double dy = yt - y;
                double fieldAngleDeg = Math.toDegrees(Math.atan2(dy, dx));
                if (fieldAngleDeg < 0) fieldAngleDeg += 360;

                distanceToTarget = Math.sqrt(dx * dx + dy * dy);
                targetAngleDeg = fieldAngleDeg - heading + 180 + AngleOffset;
                targetAngleDeg = normalizeDegrees(targetAngleDeg);

                if (hardwareInitialized && turretServo1 != null) {
                    currentServoPos = turretServo1.getPosition();
                    turretAngleDeg = servoToAngle(currentServoPos);
                }
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

    /**
     * OPTIMIZED: Faster normalization without modulo
     */
    public double normalizeDegrees(double angle) {
        if (angle > 180) {
            angle -= 360;
            if (angle > 180) angle -= 360;  // Handle extreme cases
        } else if (angle < -180) {
            angle += 360;
            if (angle < -180) angle += 360;
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
    public double getCommandedAngle() { return commandedAngle; }
}