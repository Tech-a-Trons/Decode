package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;

public class TurretOdoAiFixed implements Subsystem {

    public static final TurretOdoAiFixed INSTANCE = new TurretOdoAiFixed();

    // Hardware
    private Servo turretServo1;
    private Servo turretServo2;
    private boolean initialized = false;

    // Robot pose
    private double x = 0, y = 0, heading = 0;

    // Target
    public static double xt = 60;
    public static double yt = 60;

    // Turret state
    private double targetAngleDeg = 0;
    private double turretAngleDeg = 0;

    // PID
    public static double kP = 0.002;
    public static double kI = 0.0;
    public static double kD = 0.003;

    private double lastError = 0;
    private double integral = 0;
    private double lastTime = 0;

    // Manual mode
    public boolean manualMode = false;
    private double manualPosition = 0.4;  // Starting position

    private TurretOdoAiFixed() {}

    // ================= INIT =================
    public void init(HardwareMap hardwareMap) {
        turretServo1 = hardwareMap.get(Servo.class, "turretServo1");
        turretServo2 = hardwareMap.get(Servo.class, "turretServo2");

        turretServo1.setPosition(0.4);
        turretServo2.setPosition(0.4);
        manualPosition = 0.4;

        System.out.println("TURRET INIT CALLED");

        // Initialize lastTime to prevent dt issues on first run
        lastTime = System.nanoTime() / 1e9;

        initialized = true;
    }

    // ================= LOOP =================
    @Override
    public void periodic() {

        // Skip if in manual mode
        if (manualMode) {
            return;
        }

        // Prevent crash if init not called
        if (turretServo1 == null || turretServo2 == null) {
            System.out.println("TURRET SERVOS NULL - SKIPPING");
            return;
        }

        PedroComponent.follower().update();
        // This is already being called in TeleOp's onUpdate() method

        Pose pose = PedroComponent.follower().getPose();
        if (pose == null) {
            System.out.println("POSE NULL - SKIPPING");
            return;
        }

        // --- Update pose ---
        x = pose.getX();
        y = pose.getY();
        heading = Math.toDegrees(pose.getHeading());
        heading = (heading + 360) % 360;

        // --- Calculate target angle ---
        double fieldAngle = Math.toDegrees(Math.atan2(yt - y, xt - x));
        fieldAngle = (fieldAngle + 360) % 360;

        targetAngleDeg = normalize(fieldAngle - heading);

        // --- Read servo position ---
        double currentServoPos = turretServo1.getPosition();
        turretAngleDeg = servoToAngle(currentServoPos);

        // --- Error ---
        double error = normalize(targetAngleDeg - turretAngleDeg);

        // --- Time ---
        double currentTime = System.nanoTime() / 1e9;
        double dt = currentTime - lastTime;
        if (dt <= 0 || dt > 0.1) dt = 0.02;

        // --- PID ---
        integral += error * dt;
        double derivative = (error - lastError) / dt;

        double output =
                (kP * error) +
                        (kI * integral) +
                        (kD * derivative);

        // --- Apply ---
        double newAngle = normalize(turretAngleDeg + output * dt);
        double newPos = angleToServo(newAngle);

        turretServo1.setPosition(newPos);
        turretServo2.setPosition(newPos);

        lastError = error;
        lastTime = currentTime;
    }

    // ================= MANUAL CONTROLS =================

    public void incrementPosition(double delta) {
        manualPosition = clamp(manualPosition + delta, 0.0, 1.0);
        turretServo1.setPosition(manualPosition);
        turretServo2.setPosition(manualPosition);
    }

    public void turnRight() {
        incrementPosition(0.1);
    }

    public void turnLeft() {
        incrementPosition(-0.1);
    }

    public void setManualMode(boolean enabled) {
        manualMode = enabled;
        if (enabled) {
            // When entering manual mode, sync manual position with current servo position
            manualPosition = turretServo1.getPosition();
            System.out.println("MANUAL MODE ENABLED - Position: " + manualPosition);
        } else {
            // Reset PID state when returning to auto mode
            integral = 0;
            lastError = 0;
            lastTime = System.nanoTime() / 1e9;
            System.out.println("AUTO MODE ENABLED");
        }
    }

    public double getManualPosition() {
        return manualPosition;
    }

    // ================= Helpers =================

    private double angleToServo(double angle) {
        return clamp(1.0 - ((angle + 180) / 360.0), 0.0, 1.0);
    }

    private double servoToAngle(double pos) {
        return normalize(360.0 * (1.0 - pos) - 180.0);
    }

    private double normalize(double angle) {
        angle = (angle + 360) % 360;
        if (angle > 180) angle -= 360;
        return angle;
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}