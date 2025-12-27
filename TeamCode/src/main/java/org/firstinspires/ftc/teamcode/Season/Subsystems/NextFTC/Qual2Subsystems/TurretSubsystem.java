package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Servo;
import com.pedropathing.follower.Follower;


import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.NextFTCOpMode;

import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.hardware.impl.ServoEx;

public class TurretSubsystem implements Subsystem {
    public static final TurretSubsystem INSTANCE = new TurretSubsystem();

    private Servo turret;


    // Turret configuration
    private static final double CENTER_POSITION = 0.5;
    private static final double MIN_POSITION = 0.0;
    private static final double MAX_POSITION = 1.0;
    private static final double ROBOT_FRONT_HEADING = Math.toRadians(90); // 90 degrees in radians

    // Servo range in degrees (adjust based on your servo's actual range)
    private static final double SERVO_RANGE_DEGREES = 180.0;

    private double targetAngle = ROBOT_FRONT_HEADING;
    private boolean autoAimEnabled = false;
    private Pose targetPose = null;

    private TurretSubsystem() {
        // Private constructor for singleton
    }

        public void init() {
        if (turret == null) {
            turret = new ServoEx("turret",-0.1).getServo();
            turret.setPosition(0); // center hardware position
        }
    }

    /**
     * Update the turret - called automatically by NextFTC periodic
     */
    @Override
    public void periodic() {
        if (autoAimEnabled && targetPose != null) {
            // Get current robot pose from PedroPathing
            Pose currentPose = PedroComponent.follower().getPose();

            // Calculate angle to target
            double angleToTarget = calculateAngleToTarget(currentPose, targetPose);

            // Set turret to aim at target
            setTurretAngle(angleToTarget);
        }
    }

    /**
     * Set turret angle relative to robot's front (90 degrees)
     * @param angle Angle in radians (0 = right, PI/2 = front, PI = left)
     */
    public void setTurretAngle(double angle) {
        targetAngle = angle;

        // Calculate offset from robot front (90 degrees)
        double angleOffset = angle - ROBOT_FRONT_HEADING;

        // Normalize to [-PI, PI]
        angleOffset = normalizeAngle(angleOffset);

        // Convert to servo position [-1, 1]
        double servoPosition = angleToServoPosition(angleOffset);

        // Clamp to valid range
        servoPosition = Math.max(MIN_POSITION, Math.min(MAX_POSITION, servoPosition));

        setServoPosition(servoPosition);
    }

    /**
     * Set turret to face a specific field position
     * @param targetX Target X coordinate
     * @param targetY Target Y coordinate
     */
    public void aimAt(double targetX, double targetY) {
        Pose currentPose = PedroComponent.follower().getPose();
        targetPose = new Pose(targetX, targetY, 0);

        double angleToTarget = calculateAngleToTarget(currentPose, targetPose);
        setTurretAngle(angleToTarget);
    }

    /**
     * Enable automatic aiming at a target pose
     * @param targetX Target X coordinate
     * @param targetY Target Y coordinate
     */
    public void enableAutoAim(double targetX, double targetY) {
        targetPose = new Pose(targetX, targetY, 0);
        autoAimEnabled = true;
    }

    /**
     * Disable automatic aiming
     */
    public void disableAutoAim() {
        autoAimEnabled = false;
    }

    /**
     * Check if auto-aim is enabled
     * @return true if auto-aim is active
     */
    public boolean isAutoAimEnabled() {
        return autoAimEnabled;
    }

    /**
     * Set turret to center position (facing robot front)
     */
    public void centerTurret() {
        setTurretAngle(ROBOT_FRONT_HEADING);
    }

    /**
     * Manually set servo position
     * @param position Position from -1 (left) to 1 (right), 0 is center
     */
    public void setServoPosition(double position) {
        TurretSubsystem.INSTANCE.turret.setPosition((position + 1.0) / 2.0); // Convert [-1,1] to [0,1] for servo
    }

    /**
     * Get current turret angle
     * @return Current angle in radians
     */
    public double getTurretAngle() {
        return targetAngle;
    }

    /**
     * Get current robot pose from PedroPathing
     * @return Current robot pose
     */
    public Pose getRobotPose() {
        return PedroComponent.follower().getPose();
    }

    /**
     * Calculate angle from current pose to target pose
     */
    private double calculateAngleToTarget(Pose current, Pose target) {
        double dx = target.getX() - current.getX();
        double dy = target.getY() - current.getY();
        return Math.atan2(dy, dx);
    }

    /**
     * Convert angle offset to servo position
     * Assumes servo can rotate SERVO_RANGE_DEGREES total
     */
    private double angleToServoPosition(double angleOffset) {
        double maxAngle = Math.toRadians(SERVO_RANGE_DEGREES / 2.0);
        return angleOffset / maxAngle;
    }

    /**
     * Normalize angle to [-PI, PI]
     */
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    /**
     * Check if turret is within tolerance of target angle
     * @param tolerance Tolerance in radians
     */
    public boolean isAtTarget(double tolerance) {
        Pose current = PedroComponent.follower().getPose();
        if (targetPose == null) return true;

        double angleToTarget = calculateAngleToTarget(current, targetPose);
        return Math.abs(normalizeAngle(angleToTarget - targetAngle)) < tolerance;
    }
}
