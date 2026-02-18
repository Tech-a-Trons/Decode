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
    public static double xt = 59;
    public static double yt = 59;

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
    public static double kP = 11.00;
    public static double kI = 0.001;
    public static double kD = 0.014;

    double currentServoPos = 0;

    // Motion limits
    public static double MAX_VELOCITY = 1100.0;

    // FIX #3: TOLERANCE > 0 so integral reset actually triggers
    public static double TOLERANCE = 2.0;

    // Angle Adjust
    public static double AngleAdjust = 0;

    // ========== PID STATE VARIABLES ==========
    private double lastError = 0;
    private double integral = 0;
    private double lastUpdateTime = 0;
    private boolean firstRun = true;

    // ========== RATE LIMITING ==========
    private ElapsedTime loopTimer = new ElapsedTime();
    private static final double MIN_LOOP_TIME = 0.010;
    private int skippedLoops = 0;

    // ========== COMMANDED ANGLE TRACKING ==========
    // FIX #1 & #5: commandedAngle is always kept normalized in [-180, 180]
    private double commandedAngle = 0;
    private boolean commandedAngleInitialized = false;

    // ========== PERFORMANCE OPTIMIZATIONS ==========
    private Pose cachedPose = null;
    private static final double DEG_TO_RAD = Math.PI / 180.0;
    private static final double RAD_TO_DEG = 180.0 / Math.PI;

    private TurretOdoAi() {
    }

    // ------------------ Initialization ------------------
    public void init(HardwareMap hardwareMap) {
        try {
            turretServo1 = hardwareMap.get(Servo.class, "turretServo1");
            turretServo2 = hardwareMap.get(Servo.class, "turretServo2");

            turretServo1.setPosition(0.25);
            turretServo2.setPosition(0.25);
            manualPosition = 0.25;

            commandedAngle = servoToAngle(0.25);
            commandedAngleInitialized = true;

            loopTimer.reset();
            lastUpdateTime = loopTimer.seconds();
            firstRun = true;

            hardwareInitialized = true;

        } catch (Exception e) {
            hardwareInitialized = false;
        }
    }

    // ------------------ Manual Control ------------------
    public void turnRight() {
        AngleAdjust += 1;
    }

    public void turnLeft() {
        AngleAdjust -= 1;
    }

    // ------------------ Loop ------------------
    @Override
    public void periodic() {

        if (!hardwareInitialized) return;

        double currentTime = loopTimer.seconds();
        double timeSinceLastUpdate = currentTime - lastUpdateTime;

        if (timeSinceLastUpdate < MIN_LOOP_TIME) {
            skippedLoops++;
            return;
        }

        if (PedroComponent.follower() == null) return;

        try {
            cachedPose = PedroComponent.follower().getPose();
            if (cachedPose == null) return;

            x = cachedPose.getX() - 72;
            y = cachedPose.getY() - 72;

            double headingRad = cachedPose.getHeading();
            heading = Math.toDegrees(headingRad);
            if (heading < 0) heading += 360;

            double dx = xt - x;
            double dy = yt - y;
            distanceToTarget = Math.sqrt(dx * dx + dy * dy);

            double fieldAngleDeg = Math.toDegrees(Math.atan2(dy, dx));
            if (fieldAngleDeg < 0) fieldAngleDeg += 360;

            targetAngleDeg = fieldAngleDeg - heading + 180 + AngleOffset + AngleAdjust;
            targetAngleDeg = normalizeDegrees(targetAngleDeg);

            currentServoPos = turretServo1.getPosition();
            turretAngleDeg = servoToAngle(currentServoPos);

            if (!commandedAngleInitialized) {
                commandedAngle = turretAngleDeg;
                commandedAngleInitialized = true;
            }

            // Calculate error against normalized commandedAngle
            double error = targetAngleDeg - commandedAngle;

            // Wrap error to shortest path
            if (error > 180) error -= 360;
            else if (error < -180) error += 360;

            if (Math.abs(error) < 0.5) {
                lastUpdateTime = currentTime;
                return;
            }

            double dt = firstRun ? MIN_LOOP_TIME : timeSinceLastUpdate;
            if (dt <= 0 || dt > 0.2) dt = MIN_LOOP_TIME;
            firstRun = false;

            // === PID CALCULATION ===
            double P_output = kP * error;

            // FIX #3: Integral resets when error is within TOLERANCE (now 2.0, not 0.0)
            if (Math.abs(error) < TOLERANCE) {
                integral = 0;
            } else {
                integral += error * dt;
            }
            integral = clamp(integral, -100, 100);
            double I_output = kI * integral;

            double derivative = (error - lastError) / dt;
            double D_output = kD * derivative;

            double pidOutput = clamp(P_output + I_output + D_output, -MAX_VELOCITY, MAX_VELOCITY);

            // FIX #1 & #5: Update commandedAngle and immediately normalize it
            // so it never drifts outside [-180, 180]
            commandedAngle += pidOutput * dt;
            commandedAngle = normalizeDegrees(commandedAngle);

            double newServoPos = angleToServo(commandedAngle);
            newServoPos = clamp(newServoPos, SERVO_MIN, SERVO_MAX);

            if (Math.abs(newServoPos - currentServoPos) > 0.002) {
                turretServo1.setPosition(newServoPos);
                turretServo2.setPosition(newServoPos);
            }

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
     * FIX #2: Correct normalization using modulo — handles any magnitude input.
     */
    public double normalizeDegrees(double angle) {
        angle = angle % 360.0;
        if (angle > 180.0) angle -= 360.0;
        else if (angle < -180.0) angle += 360.0;
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
    public double AngleAdjust() { return AngleAdjust; }
    public double getManualPosition() { return manualPosition; }
    public double getCommandedAngle() { return commandedAngle; }
}