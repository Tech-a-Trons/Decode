package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.geometry.Pose;

import dev.nextftc.extensions.pedro.PedroComponent;

@Config
public class CRTurretSubsystem {
    // Dashboard tunable parameters
    public static double currentAngle = 0, targetAngle = 0, error = 0;
    public static double angleMin = -Math.PI * 2, angleMax = Math.PI * 2; // 4π range (2 full rotations each direction)
    public static double maxSpeed = 0.8; // maximum CR servo power (reduced from 1.0)
    public static double minSpeed = 0.12; // minimum power to overcome friction

    // PID gains
    public static double kP = 1.8; // proportional gain (reduced from 2.5)
    public static double kI = 0.0; // integral gain (helps eliminate steady-state error)
    public static double kD = 0.15; // derivative gain (reduces overshoot and oscillation)

    public static double errorThreshold = 0.05; // radians (about 3 degrees)
    public static double deadband = 0.03; // stop moving if error is this small (increased from 0.02)

    // Low-pass filter for encoder readings
    public static double encoderFilterAlpha = 0.7; // 0.0 = all filtering, 1.0 = no filtering

    // Acceleration limiting
    public static double maxAcceleration = 2.0; // max power change per second
    public static boolean useAccelLimit = true;

    // CR Servo and sensor hardware
    private final CRServo turretServo;
    private final AnalogInput angleEncoder; // REQUIRED for CR servo - tracks absolute position

    // State management
    public static boolean enabled = true;
    public static boolean manualMode = false;
    public static double manualPower = 0.0;

    // Odometry data
    private Pose lastRobotPose = PedroComponent.follower().getPose();
    private Pose targetPose = new Pose(122, 122, 0);

    // Encoder calibration
    public static double encoderOffsetRadians = 0.0;
    public static boolean encoderReversed = false;

    // PID state
    private double lastError = 0;
    private double integralSum = 0;
    private ElapsedTime timer = new ElapsedTime();
    private double lastTime = 0;

    // Acceleration limiting
    private double lastPower = 0;

    // Filtered angle
    private double filteredAngle = 0;
    private boolean firstRead = true;

    public CRTurretSubsystem(HardwareMap hardwareMap) {
        turretServo = hardwareMap.get(CRServo.class, "turret");
        angleEncoder = hardwareMap.get(AnalogInput.class, "turret_encoder"); // REQUIRED

        turretServo.setPower(0);
        timer.reset();
        currentAngle = readEncoderAngle();
        filteredAngle = currentAngle;
        targetAngle = currentAngle;
        firstRead = true;
    }

    public CRTurretSubsystem(CRServo turretServo, AnalogInput angleEncoder) {
        this.turretServo = turretServo;
        this.angleEncoder = angleEncoder;

        timer.reset();
        currentAngle = readEncoderAngle();
        filteredAngle = currentAngle;
        targetAngle = currentAngle;
        firstRead = true;
    }

    /**
     * Main update loop - call this in your periodic/loop method
     */
    public void periodic() {
        // Always update current angle from encoder with filtering
        double rawAngle = readEncoderAngle();
        if (firstRead) {
            filteredAngle = rawAngle;
            currentAngle = rawAngle;
            firstRead = false;
        } else {
            // Low-pass filter to reduce noise
            filteredAngle = encoderFilterAlpha * rawAngle + (1 - encoderFilterAlpha) * filteredAngle;
            currentAngle = filteredAngle;
        }

        if (!enabled) {
            turretServo.setPower(0);
            integralSum = 0;
            lastError = 0;
            lastPower = 0;
            return;
        }

        if (manualMode) {
            turretServo.setPower(manualPower);
            integralSum = 0;
            lastError = 0;
            lastPower = manualPower;
            return;
        }

        // Calculate error with shortest path
        error = normalizeAngle(targetAngle - currentAngle);

        // Stop if within deadband
        if (Math.abs(error) < deadband) {
            turretServo.setPower(0);
            integralSum = 0;
            lastError = 0;
            lastPower = 0;
            return;
        }

        // Calculate dt
        double currentTime = timer.seconds();
        double dt = currentTime - lastTime;
        lastTime = currentTime;

        // Prevent huge dt values on first run or after pauses
        if (dt > 0.1 || dt <= 0) {
            dt = 0.02; // assume 50Hz
        }

        // PID calculations
        double proportional = error * kP;

        // Integral with anti-windup
        integralSum += error * dt;
        // Clamp integral to prevent windup
        double maxIntegral = 0.5;
        integralSum = Math.max(-maxIntegral, Math.min(maxIntegral, integralSum));
        double integral = integralSum * kI;

        // Derivative (with filter on error to reduce noise)
        double derivative = 0;
        if (dt > 0) {
            derivative = ((error - lastError) / dt) * kD;
        }
        lastError = error;

        // Calculate total power
        double power = proportional + integral + derivative;

        // Clamp to max speed
        power = Math.max(-maxSpeed, Math.min(maxSpeed, power));

        // Apply minimum power threshold
        if (Math.abs(power) > 0.001 && Math.abs(power) < minSpeed) {
            power = Math.signum(power) * minSpeed;
        }

        // Acceleration limiting (optional, helps reduce jitter)
        if (useAccelLimit && dt > 0) {
            double maxDeltaPower = maxAcceleration * dt;
            double powerChange = power - lastPower;
            if (Math.abs(powerChange) > maxDeltaPower) {
                power = lastPower + Math.signum(powerChange) * maxDeltaPower;
            }
        }
        lastPower = power;

        turretServo.setPower(power);
    }

    /**
     * Set target angle in radians
     */
    public void setAngle(double radians) {
        targetAngle = Math.max(angleMin, Math.min(angleMax, radians));
        // Reset integral when target changes significantly
        if (Math.abs(radians - targetAngle) > Math.toRadians(30)) {
            integralSum = 0;
        }
    }

    /**
     * Add to current target angle
     */
    public void addAngle(double radians) {
        double newTarget = targetAngle + radians;
        targetAngle = Math.max(angleMin, Math.min(angleMax, newTarget));
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
     * Point the turret toward a target pose
     */
    public void trackTarget(Pose target, Pose robotPose) {
        this.targetPose = target;
        this.lastRobotPose = robotPose;

        double angleToTarget = Math.atan2(
                target.getY() - robotPose.getY(),
                target.getX() - robotPose.getX()
        );

        double relativeAngle = normalizeAngle(angleToTarget - robotPose.getHeading());

        // Find the closest equivalent angle within our range
        double currentNormalized = normalizeAngle(currentAngle);
        double targetCandidate = currentAngle - currentNormalized + relativeAngle;

        // Check if we need to go the "long way" around
        if (targetCandidate > angleMax) {
            targetCandidate -= Math.PI * 2;
        } else if (targetCandidate < angleMin) {
            targetCandidate += Math.PI * 2;
        }

        setAngle(targetCandidate);
    }

    /**
     * Track a moving target continuously
     */
    public void continuousTracking(Pose target, Pose robotPose) {
        trackTarget(target, robotPose);
    }

    /**
     * Read angle from absolute encoder (REQUIRED for CR servo)
     */
    private double readEncoderAngle() {
        if (angleEncoder == null) {
            throw new RuntimeException("Angle encoder is required for CRTurretSubsystem!");
        }

        double voltage = angleEncoder.getVoltage();
        double maxVoltage = angleEncoder.getMaxVoltage();
        double normalized = voltage / maxVoltage;

        // Convert to radians (0 to 2π for one rotation)
        double rawAngle = normalized * Math.PI * 2.0;

        if (encoderReversed) {
            rawAngle = Math.PI * 2.0 - rawAngle;
        }

        // Apply offset
        rawAngle += encoderOffsetRadians;

        // Handle multi-turn tracking
        // This keeps track of how many full rotations we've made
        double angleDiff = normalizeAngle(rawAngle - (currentAngle % (Math.PI * 2.0)));
        if (Math.abs(angleDiff) > Math.PI) {
            // We've crossed the 0/2π boundary
            if (angleDiff > 0) {
                // Crossed from 2π to 0 (going negative direction)
                return currentAngle - (Math.PI * 2.0 - Math.abs(angleDiff));
            } else {
                // Crossed from 0 to 2π (going positive direction)
                return currentAngle + (Math.PI * 2.0 - Math.abs(angleDiff));
            }
        } else {
            // Normal update
            return currentAngle + angleDiff;
        }
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
     * Check if turret is at target position
     */
    public boolean isReady() {
        return Math.abs(error) < errorThreshold;
    }

    /**
     * Reset to current position (makes current position the new zero)
     */
    public void reset() {
        currentAngle = 0;
        targetAngle = 0;
        encoderOffsetRadians = -readRawEncoderAngle();
        turretServo.setPower(0);
        integralSum = 0;
        lastError = 0;
        lastPower = 0;
        timer.reset();
        lastTime = 0;
    }

    /**
     * Read raw encoder angle without offset or multi-turn tracking
     */
    private double readRawEncoderAngle() {
        if (angleEncoder == null) return 0;

        double voltage = angleEncoder.getVoltage();
        double maxVoltage = angleEncoder.getMaxVoltage();
        double normalized = voltage / maxVoltage;

        double rawAngle = normalized * Math.PI * 2.0;
        if (encoderReversed) {
            rawAngle = Math.PI * 2.0 - rawAngle;
        }

        return rawAngle;
    }

    /**
     * Calibrate encoder offset to current position
     */
    public void calibrateToAngle(double angleRadians) {
        double rawAngle = readRawEncoderAngle();
        encoderOffsetRadians = angleRadians - rawAngle;
        currentAngle = angleRadians;
        filteredAngle = angleRadians;
        targetAngle = angleRadians;
        integralSum = 0;
        lastError = 0;
        firstRead = true;
    }

    /**
     * Enable/disable the turret
     */
    public void setEnabled(boolean enabled) {
        CRTurretSubsystem.enabled = enabled;
        if (!enabled) {
            integralSum = 0;
            lastError = 0;
            lastPower = 0;
        }
    }

    /**
     * Enter manual control mode
     */
    public void setManualMode(boolean manual, double power) {
        manualMode = manual;
        if (manual) {
            manualPower = Math.max(-1.0, Math.min(1.0, power));
            integralSum = 0;
            lastError = 0;
        } else {
            manualPower = 0.0;
        }
    }

    // Utility methods for common operations

    /**
     * Convenience method to reset and return this instance for chaining
     */
    public CRTurretSubsystem resetAndReturn() {
        reset();
        return this;
    }

    /**
     * Convenience method to set angle and return this instance for chaining
     */
    public CRTurretSubsystem setAngleAndReturn(double radians) {
        setAngle(radians);
        return this;
    }

    /**
     * Convenience method to enable/disable and return this instance for chaining
     */
    public CRTurretSubsystem setEnabledAndReturn(boolean enable) {
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

    /**
     * Get current servo power
     */
    public double getPower() {
        return lastPower;
    }

    /**
     * Emergency stop
     */
    public void stop() {
        turretServo.setPower(0);
        targetAngle = currentAngle;
        integralSum = 0;
        lastError = 0;
        lastPower = 0;
    }

    /**
     * Get PID integral term (for debugging)
     */
    public double getIntegral() {
        return integralSum;
    }

    /**
     * Get filtered angle (for debugging)
     */
    public double getFilteredAngle() {
        return filteredAngle;
    }
}