package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.pedropathing.geometry.Pose;

@Config
public class TurretSubsystem {
    // Dashboard tunable parameters
    public static double currentAngle = 0, targetAngle = 0, error = 0;
    public static double servoMin = 0.0, servoMax = 1.0;
    public static double angleMin = -Math.PI, angleMax = Math.PI;
    public static double trackingSpeed = 0.1; // smoothing factor for servo movement
    public static double errorThreshold = 0.05; // radians

    // Servo and sensor hardware
    private final Servo turretServo;
    private final AnalogInput angleEncoder; // optional: for absolute positioning

    // State management
    public static boolean enabled = true;
    public static boolean manualMode = false;
    public static double manualPosition = 0.5;

    // Odometry data
    private Pose lastRobotPose = new Pose(0, 0, 0);
    private Pose targetPose = new Pose(0, 0, 0);

    public TurretSubsystem(HardwareMap hardwareMap) {
        turretServo = hardwareMap.get(Servo.class, "turret");
        angleEncoder = null; // hardwareMap.get(AnalogInput.class, "angle_encoder"); // optional

        turretServo.setPosition(0.5); // center position
        currentAngle = 0;
        targetAngle = 0;
    }

    public TurretSubsystem(Servo turretServo, AnalogInput angleEncoder) {
        this.turretServo = turretServo;
        this.angleEncoder = angleEncoder;
    }

    /**
     * Main update loop - call this in your periodic/loop method
     */
    public void periodic() {
        if (!enabled) {
            turretServo.setPosition(0.5);
            return;
        }

        if (manualMode) {
            turretServo.setPosition(manualPosition);
            currentAngle = servoPositionToAngle(manualPosition);
            return;
        }

        // Calculate error
        error = normalizeAngle(targetAngle - currentAngle);

        // Smooth servo movement
        double targetServoPos = angleToServoPosition(targetAngle);
        double currentServoPos = turretServo.getPosition();
        double newServoPos = currentServoPos + (targetServoPos - currentServoPos) * trackingSpeed;

        // Clamp to servo limits
        newServoPos = Math.max(servoMin, Math.min(servoMax, newServoPos));

        turretServo.setPosition(newServoPos);
        currentAngle = servoPositionToAngle(newServoPos);

        // Optional: read from absolute encoder if available
        if (angleEncoder != null) {
            currentAngle = readEncoderAngle();
        }
    }

    /**
     * Set target angle in radians
     */
    public void setAngle(double radians) {
        targetAngle = normalizeAngle(radians);
    }

    /**
     * Add to current target angle
     */
    public void addAngle(double radians) {
        targetAngle = normalizeAngle(targetAngle + radians);
    }

    /**
     * Get current angle in radians
     */
    public double getAngle() {
        return currentAngle;
    }

    /**
     * Get target angle in radians
     */
    public double getTargetAngle() {
        return targetAngle;
    }

    /**
     * Point the odometry pod toward a target pose
     */
    public void trackTarget(Pose target, Pose robotPose) {
        this.targetPose = target;
        this.lastRobotPose = robotPose;

        double angleToTarget = Math.atan2(
                target.getY() - robotPose.getY(),
                target.getX() - robotPose.getX()
        );

        double relativeAngle = normalizeAngle(angleToTarget - robotPose.getHeading());
        setAngle(relativeAngle);
    }

    /**
     * Track a moving target continuously
     */
    public void continuousTracking(Pose target, Pose robotPose) {
        trackTarget(target, robotPose);
    }

    /**
     * Convert servo position (0-1) to angle in radians
     */
    private double servoPositionToAngle(double servoPos) {
        double normalized = (servoPos - servoMin) / (servoMax - servoMin);
        return angleMin + normalized * (angleMax - angleMin);
    }

    /**
     * Convert angle in radians to servo position (0-1)
     */
    private double angleToServoPosition(double angle) {
        double normalized = (angle - angleMin) / (angleMax - angleMin);
        return servoMin + normalized * (servoMax - servoMin);
    }

    /**
     * Read angle from absolute encoder (if available)
     */
    private double readEncoderAngle() {
        if (angleEncoder == null) return currentAngle;

        double voltage = angleEncoder.getVoltage();
        double maxVoltage = angleEncoder.getMaxVoltage();
        double normalized = voltage / maxVoltage;

        return angleMin + normalized * (angleMax - angleMin);
    }

    /**
     * Normalize angle to [-PI, PI]
     */
    public static double normalizeAngle(double angleRadians) {
        double angle = angleRadians % (Math.PI * 2.0);
        if (angle <= -Math.PI) angle += Math.PI * 2.0;
        if (angle > Math.PI) angle -= Math.PI * 2.0;
        return angle;
    }

    /**
     * Check if tracker is at target position
     */
    public boolean isReady() {
        return Math.abs(error) < errorThreshold;
    }

    /**
     * Reset to center position
     */
    public void reset() {
        currentAngle = 0;
        targetAngle = 0;
        turretServo.setPosition(0.5);
    }

    /**
     * Enable/disable the tracker
     */
    public void setEnabled(boolean enabled) {
        TurretSubsystem.enabled = enabled;
    }

    /**
     * Enter manual control mode
     */
    public void setManualMode(boolean manual, double position) {
        manualMode = manual;
        if (manual) {
            manualPosition = Math.max(servoMin, Math.min(servoMax, position));
        }
    }

    // Utility methods for common operations

    /**
     * Convenience method to reset and return this instance for chaining
     */
    public TurretSubsystem resetAndReturn() {
        reset();
        return this;
    }

    /**
     * Convenience method to set angle and return this instance for chaining
     */
    public TurretSubsystem setAngleAndReturn(double radians) {
        setAngle(radians);
        return this;
    }

    /**
     * Convenience method to enable/disable and return this instance for chaining
     */
    public TurretSubsystem setEnabledAndReturn(boolean enable) {
        setEnabled(enable);
        return this;
    }

    /**
     * Get current tracking error in radians
     */
    public double getError() {
        return error;
    }

    /**
     * Get the last tracked target pose
     */
    public Pose getTargetPose() {
        return targetPose;
    }

    /**
     * Get the last robot pose used for tracking
     */
    public Pose getLastRobotPose() {
        return lastRobotPose;
    }
}