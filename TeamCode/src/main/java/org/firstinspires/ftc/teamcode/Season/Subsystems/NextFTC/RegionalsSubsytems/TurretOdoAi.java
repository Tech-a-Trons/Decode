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

    // Servo safety
    public static double SERVO_MIN = 0.0;
    public static double SERVO_MAX = 1.0;
    public boolean hardwareInitialized = false;

    // ========== PID CONSTANTS ==========
    public static double kP = 9.00;
    public static double kI = 0.000;
    public static double kD = 0.012;

    double currentServoPos = 0;

    // Motion limits
    public static double MAX_VELOCITY = 500.0;
    public static double TOLERANCE = 2.0;

    // ========== PID STATE VARIABLES ==========
    private double lastError = 0;
    private double integral = 0;
    private double lastUpdateTime = 0;
    private boolean firstRun = true;

    // ========== RATE LIMITING ==========
    private ElapsedTime loopTimer = new ElapsedTime();
    private static final double MIN_LOOP_TIME = 0.01; // 20ms minimum between updates (50Hz max)
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

            // Initialize timers
            loopTimer.reset();
            lastUpdateTime = loopTimer.seconds();
            firstRun = true;

            hardwareInitialized = true;

        } catch (Exception e) {
            hardwareInitialized = false;
        }
    }

    // ------------------ Loop ------------------
    @Override
    public void periodic() {
        // === 0. RATE LIMITING ===
        double currentTime = loopTimer.seconds();
        double timeSinceLastUpdate = currentTime - lastUpdateTime;

        // Skip if called too frequently (less than 20ms since last update)
        if (timeSinceLastUpdate < MIN_LOOP_TIME) {
            skippedLoops++;
            return;
        }

        // === 1. HARDWARE CHECK ===
        if (!hardwareInitialized || turretServo1 == null || turretServo2 == null) {
            return;
        }

        try {
            // === 2. SAFETY CHECKS ===
            if (PedroComponent.follower() == null) {
                return;
            }

            Pose currentPose = PedroComponent.follower().getPose();
            if (currentPose == null) {
                return;
            }

            // === 3. UPDATE ROBOT POSE ===
            x = currentPose.getX() - 72;
            y = currentPose.getY() - 72;
            heading = Math.toDegrees(currentPose.getHeading());
            heading = (heading + 360) % 360;

            // === 4. CALCULATE TARGET ANGLE ===
            double fieldAngleDeg = Math.toDegrees(Math.atan2(yt - y, xt - x));
            fieldAngleDeg = (fieldAngleDeg + 360) % 360;

            distanceToTarget = Math.hypot(xt - x, yt - y);

            targetAngleDeg = fieldAngleDeg - heading + 180;
            targetAngleDeg = normalizeDegrees(targetAngleDeg);

            // === 5. READ CURRENT TURRET POSITION ===
            currentServoPos = turretServo1.getPosition();
            turretAngleDeg = servoToAngle(currentServoPos);

            // === 6. CALCULATE ERROR ===
            double error = targetAngleDeg - turretAngleDeg;

            // Wrap error to -180 to +180
            if (error > 180) error -= 360;
            if (error < -180) error += 360;

            // === 7. CALCULATE TIME DELTA ===
            double dt = timeSinceLastUpdate;

            if (firstRun) {
                dt = MIN_LOOP_TIME;
                firstRun = false;
            }

            // Clamp dt to reasonable values
            if (dt <= 0 || dt > 0.2) {
                dt = MIN_LOOP_TIME;
            }

            // === 8. PID CALCULATIONS ===
            double P_output = kP * error;

            integral += error * dt;

            if (Math.abs(error) < TOLERANCE) {
                integral = 0;
            }

            integral = clamp(integral, -100, 100);
            double I_output = kI * integral;

            double derivative = (error - lastError) / dt;
            double D_output = kD * derivative;

            // === 9. COMBINE PID OUTPUTS ===
            double pidOutput = P_output + I_output + D_output;
            pidOutput = clamp(pidOutput, -MAX_VELOCITY, MAX_VELOCITY);

            // === 10. UPDATE SERVO POSITION ===
            double positionChange = pidOutput * dt;
            double newAngle = turretAngleDeg + positionChange;
            newAngle = normalizeDegrees(newAngle);

            double newServoPos = angleToServo(newAngle);
            newServoPos = clamp(newServoPos, SERVO_MIN, SERVO_MAX);

            // Only update servos if position changed significantly (reduce bus traffic)
            if (Math.abs(newServoPos - currentServoPos) > 0.001) {
                turretServo1.setPosition(newServoPos);
                turretServo2.setPosition(newServoPos);
            }

            // === 11. UPDATE STATE ===
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
}