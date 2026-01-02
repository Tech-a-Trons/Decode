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

    // Gear ratio: servo 80T to output 200T = 2.5x mechanical advantage
    public static double gearRatio = 200.0 / 80.0; // 2.5

    // Physical limits in DEGREES (easier to tune)
    // Center is 0°, adjust these to match your physical hard stops
    public static double negativeRangeMax = -180; // degrees - how far CCW from center
    public static double positiveRangeMax = 20;  // degrees - how far CW from center

    // Calculated limits in radians (don't modify these directly)
    public static double angleMin = Math.toRadians(negativeRangeMax);
    public static double angleMax = Math.toRadians(positiveRangeMax);


    // Wrap threshold - triggers wrap when this close to limits (in degrees)
    public static double wrapMarginDegrees = 20;
    public static double posWrapThreshold = Math.toRadians(positiveRangeMax - wrapMarginDegrees);
    public static double negWrapThreshold = Math.toRadians(negativeRangeMax + wrapMarginDegrees);// start wrapping 20° before hitting hard stop
    public static double wrapThreshold = Math.toRadians(Math.max(
            Math.abs(positiveRangeMax) - wrapMarginDegrees,
            Math.abs(negativeRangeMax) - wrapMarginDegrees
    ));


    public static double maxSpeed = 0.8; // maximum CR servo power
    public static double minSpeed = 0.12; // minimum power to overcome friction

    // PID gains
    public static double kP = 1.8;
    public static double kI = 0.0;
    public static double kD = 0.15;

    public static double errorThreshold = 0.05; // radians (about 3 degrees)
    public static double deadband = 0.03;

    // Low-pass filter for encoder readings
    public static double encoderFilterAlpha = 0.7;

    // Acceleration limiting
    public static double maxAcceleration = 2.0;
    public static boolean useAccelLimit = true;

    // CR Servo and sensor hardware
    private final CRServo turretServo;
    private final AnalogInput angleEncoder; // REQUIRED - tracks absolute position

    // State management
    public static boolean enabled = true;
    public static boolean manualMode = false;
    public static double manualPower = 0.0;

    // Target tracking
    private Pose lastRobotPose = new Pose(0, 0, 0);
    private Pose targetPose = new Pose(122, 122, 0);

    // Turret offset on robot (relative to robot center)
    public static double turretOffsetX = 0.0; // inches from robot center
    public static double turretOffsetY = 0.0; // inches from robot center

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

    // Wrapping state
    private int wraps = 0; // how many times we've wrapped around

    public CRTurretSubsystem(HardwareMap hardwareMap) {
        turretServo = hardwareMap.get(CRServo.class, "turret");
        angleEncoder = hardwareMap.get(AnalogInput.class, "turret_encoder");

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

        // Check if we need to wrap around to avoid hitting limits
        handleWrapping();

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
        double maxIntegral = 0.5;
        integralSum = Math.max(-maxIntegral, Math.min(maxIntegral, integralSum));
        double integral = integralSum * kI;

        // Derivative
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

        // Acceleration limiting
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
     * Handle intelligent wrapping to avoid hitting hard stops
     */
    private void handleWrapping() {
        // Recalculate limits in case they were changed via dashboard
        angleMin = Math.toRadians(negativeRangeMax);
        angleMax = Math.toRadians(positiveRangeMax);
        wrapThreshold = Math.toRadians(Math.max(
                Math.abs(positiveRangeMax) - wrapMarginDegrees,
                Math.abs(negativeRangeMax) - wrapMarginDegrees
        ));

        // Check if we're approaching limits
        if (currentAngle > wrapThreshold) {
            // Near positive limit - wrap to negative side
            double equivalentAngle = currentAngle - (Math.PI * 2);
            if (equivalentAngle >= angleMin) {
                // Adjust target to equivalent angle on the other side
                double targetNormalized = normalizeAngle(targetAngle);
                targetAngle = equivalentAngle - normalizeAngle(currentAngle) + targetNormalized;
                wraps--;
            }
        } else if (currentAngle < -wrapThreshold) {
            // Near negative limit - wrap to positive side
            double equivalentAngle = currentAngle + (Math.PI * 2);
            if (equivalentAngle <= angleMax) {
                // Adjust target to equivalent angle on the other side
                double targetNormalized = normalizeAngle(targetAngle);
                targetAngle = equivalentAngle - normalizeAngle(currentAngle) + targetNormalized;
                wraps++;
            }
        }
    }

    /**
     * Set target angle in radians (output shaft angle)
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
     * Get current angle in radians (output shaft angle)
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
     * Point the turret toward a target pose using robot X, Y, and heading
     */
    public void trackTarget(Pose target, Pose robotPose) {
        this.targetPose = target;
        this.lastRobotPose = robotPose;

        // Calculate turret's actual position on the field (accounting for offset from robot center)
        double cosHeading = Math.cos(robotPose.getHeading());
        double sinHeading = Math.sin(robotPose.getHeading());

        double turretWorldX = robotPose.getX() + (turretOffsetX * cosHeading - turretOffsetY * sinHeading);
        double turretWorldY = robotPose.getY() + (turretOffsetX * sinHeading + turretOffsetY * cosHeading);

        // Calculate angle from turret to target in world frame
        double angleToTarget = Math.atan2(
                target.getY() - turretWorldY,
                target.getX() - turretWorldX
        );

        // Convert to robot-relative angle (this is what the turret needs to point at)
        double relativeAngle = normalizeAngle(angleToTarget - robotPose.getHeading());

        // Find the best target angle considering our current position and wrapping
        setOptimalTargetAngle(relativeAngle);
    }

    /**
     * Set target angle intelligently, considering current position and wrapping
     */
    private void setOptimalTargetAngle(double desiredAngle) {
        // Normalize desired angle to [-π, π]
        desiredAngle = normalizeAngle(desiredAngle);

        // Find the closest equivalent angle to our current position
        // Options: desiredAngle, desiredAngle ± 2π, etc.
        double option1 = desiredAngle;
        double option2 = desiredAngle + Math.PI * 2;
        double option3 = desiredAngle - Math.PI * 2;

        // Choose the option that requires minimum rotation and stays within limits
        double[] options = {option1, option2, option3};
        double bestOption = option1;
        double minDistance = Double.MAX_VALUE;

        for (double option : options) {
            if (option >= angleMin && option <= angleMax) {
                double distance = Math.abs(option - currentAngle);
                if (distance < minDistance) {
                    minDistance = distance;
                    bestOption = option;
                }
            }
        }

        setAngle(bestOption);
    }

    /**
     * Track a moving target continuously
     */
    public void continuousTracking(Pose target, Pose robotPose) {
        trackTarget(target, robotPose);
    }

    /**
     * Read angle from absolute encoder with gear ratio applied
     */
    private double readEncoderAngle() {
        if (angleEncoder == null) {
            throw new RuntimeException("Angle encoder is required for CRTurretSubsystem!");
        }

        double voltage = angleEncoder.getVoltage();
        double maxVoltage = angleEncoder.getMaxVoltage();
        double normalized = voltage / maxVoltage;

        // Convert to servo angle (0 to 2π for one servo rotation)
        double servoAngle = normalized * Math.PI * 2.0;

        if (encoderReversed) {
            servoAngle = Math.PI * 2.0 - servoAngle;
        }

        // Apply offset
        servoAngle += encoderOffsetRadians;

        // Convert from servo angle to output shaft angle using gear ratio
        // Output rotates 2.5x faster than servo input
        double rawAngle = servoAngle * gearRatio;

        // Handle multi-turn tracking
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
        wraps = 0;
    }

    /**
     * Read raw encoder angle without offset, multi-turn tracking, or gear ratio
     */
    private double readRawEncoderAngle() {
        if (angleEncoder == null) return 0;

        double voltage = angleEncoder.getVoltage();
        double maxVoltage = angleEncoder.getMaxVoltage();
        double normalized = voltage / maxVoltage;

        double servoAngle = normalized * Math.PI * 2.0;
        if (encoderReversed) {
            servoAngle = Math.PI * 2.0 - servoAngle;
        }

        return servoAngle;
    }

    /**
     * Calibrate encoder offset to current position
     */
    public void calibrateToAngle(double angleRadians) {
        double rawAngle = readRawEncoderAngle();
        // Divide by gear ratio since we're calibrating the servo angle
        encoderOffsetRadians = (angleRadians / gearRatio) - rawAngle;
        currentAngle = angleRadians;
        filteredAngle = angleRadians;
        targetAngle = angleRadians;
        integralSum = 0;
        lastError = 0;
        firstRead = true;
        wraps = 0;
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

    // Utility methods

    public CRTurretSubsystem resetAndReturn() {
        reset();
        return this;
    }

    public CRTurretSubsystem setAngleAndReturn(double radians) {
        setAngle(radians);
        return this;
    }

    public CRTurretSubsystem setEnabledAndReturn(boolean enable) {
        setEnabled(enable);
        return this;
    }

    public double getError() {
        return error;
    }

    public Pose getTargetPose() {
        return targetPose;
    }

    public Pose getLastRobotPose() {
        return lastRobotPose;
    }

    public double getPower() {
        return lastPower;
    }

    public void stop() {
        turretServo.setPower(0);
        targetAngle = currentAngle;
        integralSum = 0;
        lastError = 0;
        lastPower = 0;
    }

    public double getIntegral() {
        return integralSum;
    }

    public double getFilteredAngle() {
        return filteredAngle;
    }

    public int getWraps() {
        return wraps;
    }

    /**
     * Get the output shaft angle in degrees (for debugging)
     */
    public double getAngleDegrees() {
        return Math.toDegrees(currentAngle);
    }

    /**
     * Get the servo angle in degrees (before gear ratio)
     */
    public double getServoAngleDegrees() {
        return Math.toDegrees(currentAngle / gearRatio);
    }
}