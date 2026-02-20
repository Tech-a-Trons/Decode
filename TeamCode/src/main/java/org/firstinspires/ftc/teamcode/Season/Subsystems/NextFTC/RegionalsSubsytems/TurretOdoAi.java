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

    public double AngleAdjust = 0;

    // ------------------ Target (Red Goal) ------------------
    public static double xt = 130;
    public static double yt = 130;

    double AngleOffset = -30;


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
    public static double kP = 10.00;
    public static double kI = 0.001;
    public static double kD = 0.04;

    double currentServoPos = 0;

    // Motion limits
    public static double MAX_VELOCITY = 1100;
    public static double TOLERANCE = 0.1;

    // ========== PID STATE VARIABLES ==========
    private double lastError = 0;
    private double integral = 0;
    private double lastUpdateTime = 0;
    private boolean firstRun = true;

    // ========== RATE LIMITING ==========
    private ElapsedTime loopTimer = new ElapsedTime();
    private static final double MIN_LOOP_TIME = 0.010;
    private int skippedLoops = 0;

    // ========== IMPROVED WRAPPING FIX ==========
    private double commandedAngle = 0;
    private boolean commandedAngleInitialized = false;

    // ========== PERFORMANCE OPTIMIZATIONS ==========
    private Pose cachedPose = null;
    private static final double DEG_TO_RAD = Math.PI / 180.0;
    private static final double RAD_TO_DEG = 180.0 / Math.PI;

    private TurretOdoAi() {
    }

    public void setAlliance(String alliance) {
        if (alliance.equals("blue")) {
            AngleOffset = -30 + 90;
        }
        if (alliance.equals("red")) {
            AngleOffset = -36;
            xt = 130;
            yt = 130;
        }
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

            // Initialize commanded angle tracking — normalize on init
            commandedAngle = normalizeDegrees(servoToAngle(0.25));
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

            // Update commanded angle when manually moving — normalize to prevent drift
            commandedAngle = normalizeDegrees(servoToAngle(manualPosition));
        }
    }


    // ------------------ Loop ------------------
    @Override
    public void periodic() {
        // === EARLY EXIT: Manual mode lightweight update ===
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
            distanceToTarget = Math.sqrt(dx * dx + dy * dy);

            double fieldAngleDeg = Math.toDegrees(Math.atan2(dy, dx));
            if (fieldAngleDeg < 0) fieldAngleDeg += 360;

            targetAngleDeg = fieldAngleDeg - heading + 180 + AngleOffset + AngleAdjust;
            targetAngleDeg = normalizeDegrees(targetAngleDeg);

            // === READ CURRENT POSITION ===
            currentServoPos = turretServo1.getPosition();
            turretAngleDeg = servoToAngle(currentServoPos);

            // === Initialize commanded angle if needed — always normalized ===
            if (!commandedAngleInitialized) {
                commandedAngle = normalizeDegrees(turretAngleDeg);
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

            // === UPDATE POSITION — normalize commandedAngle to prevent unbounded drift ===
            commandedAngle += pidOutput * dt;
            commandedAngle = normalizeDegrees(commandedAngle);  // FIX: keep commandedAngle bounded
            double newServoPos = angleToServo(commandedAngle);  // no need for separate normalizedAngle
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
//    private void updateManualModeTelemetry() {
//        if (PedroComponent.follower() != null) {
//            Pose currentPose = PedroComponent.follower().getPose();
//            if (currentPose != null) {
//                x = currentPose.getX() - 72;
//                y = currentPose.getY() - 72;
//                heading = Math.toDegrees(currentPose.getHeading());
//                if (heading < 0) heading += 360;
//
//                double dx = xt - x;
//                double dy = yt - y;
//                double fieldAngleDeg = Math.toDegrees(Math.atan2(dy, dx));
//                if (fieldAngleDeg < 0) fieldAngleDeg += 360;
//
//                distanceToTarget = Math.sqrt(dx * dx + dy * dy);
//                targetAngleDeg = fieldAngleDeg - heading + 180 + AngleOffset + AngleAdjust;
//                targetAngleDeg = normalizeDegrees(targetAngleDeg);
//
//                if (hardwareInitialized && turretServo1 != null) {
//                    currentServoPos = turretServo1.getPosition();
//                    turretAngleDeg = servoToAngle(currentServoPos);
//                }



    // ------------------ Helper Functions ------------------
    public void turnRight() {
        AngleAdjust += 2;
    }

    public void turnLeft() {
        AngleAdjust -= 2;
    }

    private double angleToServo(double angleDeg) {
        angleDeg = normalizeDegrees(angleDeg);
        double pos = 1 - ((angleDeg + 180) / 340);
        return clamp(pos, SERVO_MIN, SERVO_MAX);
    }

    private double servoToAngle(double servoPos) {
        double angle = 340 * (1.0 - servoPos) - 180.0;
        return normalizeDegrees(angle);
    }

    /**
     * OPTIMIZED: Faster normalization without modulo
     */
    public double normalizeDegrees(double angle) {
        if (angle > 180) {
            angle -= 340;
            if (angle > 180) angle -= 340;
        } else if (angle < -180) {
            angle += 340;
            if (angle < -180) angle += 340;
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