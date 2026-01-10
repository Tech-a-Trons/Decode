package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems.RedExperimentalDistanceLExtractor;

@Config
public class VisionEncoderTurret {
    private CRServo turretServo;
    private AnalogInput turretEncoder;
    private RedExperimentalDistanceLExtractor limelight;
    private Telemetry telemetry;

    // Encoder constants
    private static final double ENCODER_VOLTAGE_RANGE = 3.3; // Axon Max outputs 0-3.3V
    private static final double DEGREES_PER_REVOLUTION = 360.0;

    // Gear ratio: servo is 80 tooth, turret is 200 tooth
    private static final double GEAR_RATIO = 200.0 / 80.0; // 2.5:1

    // Angle limits (in degrees)
    public static double MIN_ANGLE = -180.0;
    public static double MAX_ANGLE = 180.0;

    // PID coefficients - Tuned for smooth, non-overshooting control
    public static double kP = 0.013;     // Reduced from 0.03 - prevents aggressive overshooting
    public static double kI = 0.0001;    // Very low to prevent integral windup
    public static double kD = 0.002;     // Adds damping to reduce overshoot
    public static double kF = 0.0;       // Feedforward if needed

    // Vision-to-angle conversion
    public static double VISION_TX_TO_ANGLE_SCALE = 1.0; // Degrees of turret per degree of tx

    // Control parameters with error-based scaling
    public static double POSITION_TOLERANCE = 2.0; // degrees - stop when within this range
    public static double SLOW_DOWN_THRESHOLD = 20; // degrees - start slowing down here

    // Power scaling based on error
    public static double MAX_SERVO_POWER = 1;      // Reduced from 0.8 - prevents wild swings
    public static double MIN_SERVO_POWER = 0.03;     // Minimum power to overcome friction
    public static double SLOW_POWER_MULTIPLIER = 0.3; // Power multiplier when close to target

    // Velocity limits
    public static double MAX_VELOCITY = 120.0; // degrees/second - reduced for smoother motion
    public static double MAX_ACCELERATION = 200.0; // degrees/second^2

    // PID variables
    private double integralSum = 0;
    private double lastError = 0;
    private long lastUpdateTime = 0;

    // State tracking
    private double currentAngle = 0;
    private double targetAngle = 0;
    private double encoderOffset = 0;
    private double lastVelocity = 0;

    private boolean isAligning = false;

    public VisionEncoderTurret(HardwareMap hardwareMap, RedExperimentalDistanceLExtractor limelight) {
        this.turretServo = hardwareMap.get(CRServo.class, "turret");
        this.turretEncoder = hardwareMap.get(AnalogInput.class, "turret_encoder");
        this.limelight = limelight;

        calibrateEncoder();
        lastUpdateTime = System.nanoTime();
    }

    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    /**
     * Calibrate the encoder to set current position as reference
     */
    public void calibrateEncoder() {
        encoderOffset = getRawEncoderAngle();
    }

    /**
     * Get raw encoder angle from analog voltage (0-360 degrees)
     */
    private double getRawEncoderAngle() {
        double voltage = turretEncoder.getVoltage();
        double angle = (voltage / ENCODER_VOLTAGE_RANGE) * DEGREES_PER_REVOLUTION;
        return angle;
    }

    /**
     * Get current turret angle accounting for gear ratio and offset
     */
    public double getCurrentAngle() {
        double rawAngle = getRawEncoderAngle();

        // Account for encoder offset and wraparound
        double deltaAngle = rawAngle - encoderOffset;
        if (deltaAngle > 180) deltaAngle -= 360;
        if (deltaAngle < -180) deltaAngle += 360;

        // Apply gear ratio
        currentAngle = deltaAngle * GEAR_RATIO;

        return currentAngle;
    }

    /**
     * Calculate target angle from Limelight tx value
     */
    private double calculateTargetFromVision() {
        Double tx = limelight.getTx();

        if (tx == null) {
            return currentAngle; // Hold current position if no target
        }

        // Convert tx error to target angle
        // Negative because if target is right (+tx), we need to turn right (increase angle)
        double angleError = -tx * VISION_TX_TO_ANGLE_SCALE;
        double newTarget = currentAngle + angleError;

        // Clamp to mechanical limits
        newTarget = Range.clip(newTarget, MIN_ANGLE, MAX_ANGLE);

        return newTarget;
    }

    /**
     * Scale power based on error magnitude (slow down when close)
     */
    private double scalePowerByError(double error, double basePower) {
        double absError = Math.abs(error);

        // If very close, use slow power
        if (absError <= POSITION_TOLERANCE) {
            return 0; // Stop
        }

        // Linear scaling between POSITION_TOLERANCE and SLOW_DOWN_THRESHOLD
        if (absError <= SLOW_DOWN_THRESHOLD) {
            double scale = (absError - POSITION_TOLERANCE) / (SLOW_DOWN_THRESHOLD - POSITION_TOLERANCE);
            scale = Range.clip(scale, 0, 1);
            // Scale between MIN and SLOW power
            double slowPower = MIN_SERVO_POWER + (MAX_SERVO_POWER * SLOW_POWER_MULTIPLIER - MIN_SERVO_POWER) * scale;
            return Math.signum(basePower) * slowPower;
        }

        // Far from target - use full power but clamped
        return basePower;
    }

    /**
     * PID controller with velocity limiting and error-based power scaling
     */
    private double calculatePID(double error) {
        long currentTime = System.nanoTime();
        double deltaTime = (currentTime - lastUpdateTime) / 1e9; // Convert to seconds
        lastUpdateTime = currentTime;

        if (deltaTime <= 0) deltaTime = 0.001; // Prevent divide by zero

        // Proportional
        double p = kP * error;

        // Integral with anti-windup (only accumulate when error is moderate)
        if (Math.abs(error) < 20.0) {
            integralSum += error * deltaTime;
            integralSum = Range.clip(integralSum, -50, 50); // Tighter limit
        } else {
            integralSum *= 0.95; // Decay integral when error is large
        }
        double i = kI * integralSum;

        // Derivative with smoothing
        double derivative = (error - lastError) / deltaTime;
        double d = kD * derivative;
        lastError = error;

        // Feedforward
        double f = kF * Math.signum(error);

        // Calculate base output
        double baseOutput = p + i + d + f;

        // Apply error-based power scaling (slow down when close)
        double scaledOutput = scalePowerByError(error, baseOutput);

        // Apply minimum power threshold
        if (Math.abs(scaledOutput) > 0.001 && Math.abs(scaledOutput) < MIN_SERVO_POWER) {
            scaledOutput = Math.signum(scaledOutput) * MIN_SERVO_POWER;
        }

        return Range.clip(scaledOutput, -MAX_SERVO_POWER, MAX_SERVO_POWER);
    }

    /**
     * Align turret using Limelight vision with encoder feedback
     */
    public void align() {
        // Update current angle from encoder
        getCurrentAngle();

        // Calculate target from vision
        targetAngle = calculateTargetFromVision();

        // Check if we have a valid target
        if (limelight.getTx() == null) {
            stopTurret();
            return;
        }

        isAligning = true;

        // Calculate error
        double error = targetAngle - currentAngle;

        // If within tolerance, stop
        if (Math.abs(error) <= POSITION_TOLERANCE) {
            stopTurret();
            return;
        }

        // Calculate PID output with error-based scaling
        double power = calculatePID(error);

        // Set servo power
        turretServo.setPower(power);
    }

    /**
     * Close alignment mode - more careful approach
     */
    public void closeAlign() {
        // Temporarily adjust parameters for close range
        double originalKp = kP;
        double originalMaxPower = MAX_SERVO_POWER;
        double originalSlowThreshold = SLOW_DOWN_THRESHOLD;

        kP = 0.018; // Slightly higher for responsiveness
        MAX_SERVO_POWER = 0.4; // Lower max power for precision
        SLOW_DOWN_THRESHOLD = 8.0; // Start slowing down earlier

        align();

        // Restore original values
        kP = originalKp;
        MAX_SERVO_POWER = originalMaxPower;
        SLOW_DOWN_THRESHOLD = originalSlowThreshold;
    }

    /**
     * Far alignment mode - standard PID
     */
    public void farAlign() {
        align();
    }

    /**
     * Stop the turret
     */
    public void stopTurret() {
        turretServo.setPower(0);
        isAligning = false;
        lastVelocity = 0;
        integralSum = 0; // Reset integral when stopping
    }

    /**
     * Reset PID controller
     */
    public void resetPID() {
        integralSum = 0;
        lastError = 0;
        lastVelocity = 0;
        lastUpdateTime = System.nanoTime();
    }

    /**
     * Check if turret is aligned with target
     */
    public boolean isAligned() {
        Double tx = limelight.getTx();
        return tx != null && Math.abs(tx) <= POSITION_TOLERANCE;
    }

    /**
     * Check if turret is attempting to align
     */
    public boolean isAligning() {
        return isAligning;
    }

    /**
     * Manually set turret power (for testing)
     */
    public void setTurretPower(double power) {
        isAligning = false;
        turretServo.setPower(Range.clip(power, -MAX_SERVO_POWER, MAX_SERVO_POWER));
    }

    /**
     * Update telemetry with detailed alignment info
     */
    public void updateTelemetry() {
        if (telemetry != null) {
            Double tx = limelight.getTx();
            double error = targetAngle - currentAngle;
            telemetry.addData("--- Vision+Encoder Turret ---", "");
            telemetry.addData("Target Visible", limelight.isTargetVisible());
            telemetry.addData("Vision TX", tx != null ? String.format("%.2f°", tx) : "N/A");
            telemetry.addData("Current Angle", String.format("%.2f°", currentAngle));
            telemetry.addData("Target Angle", String.format("%.2f°", targetAngle));
            telemetry.addData("Error", String.format("%.2f°", error));
            telemetry.addData("Error Zone", Math.abs(error) <= POSITION_TOLERANCE ? "ALIGNED" :
                    Math.abs(error) <= SLOW_DOWN_THRESHOLD ? "SLOWING" : "FULL SPEED");
            telemetry.addData("Is Aligned", isAligned());
            telemetry.addData("Is Aligning", isAligning);
            telemetry.addData("Encoder Voltage", String.format("%.3fV", turretEncoder.getVoltage()));
        }
    }

    /**
     * Get telemetry string for debugging
     */
    public String getTelemetry() {
        Double tx = limelight.getTx();
        double error = targetAngle - currentAngle;
        return String.format(
                "Vision+Encoder Turret:\n" +
                        "  Current: %.2f°\n" +
                        "  Target: %.2f°\n" +
                        "  Error: %.2f°\n" +
                        "  Vision TX: %s\n" +
                        "  Aligned: %b\n" +
                        "  Zone: %s",
                currentAngle,
                targetAngle,
                error,
                tx != null ? String.format("%.2f°", tx) : "N/A",
                isAligned(),
                Math.abs(error) <= POSITION_TOLERANCE ? "ALIGNED" :
                        Math.abs(error) <= SLOW_DOWN_THRESHOLD ? "SLOWING" : "FULL"
        );
    }
}