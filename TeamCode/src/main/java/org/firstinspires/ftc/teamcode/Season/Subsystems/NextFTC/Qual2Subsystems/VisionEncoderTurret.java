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
    public static double MAX_ANGLE =180.0;

    // PID coefficients - Much higher than open-loop! Tune these via FTC Dashboard
    public static double kP = 0.03;      // Start here, can go even higher
    public static double kI = 0.0005;    // Eliminates steady-state error
    public static double kD = 0.008;     // Reduces overshoot
    public static double kF = 0.0;       // Feedforward if needed

    // Vision-to-angle conversion
    public static double VISION_TX_TO_ANGLE_SCALE = 1.0; // Degrees of turret per degree of tx

    // Control parameters
    public static double POSITION_TOLERANCE = 2.0; // degrees
    public static double MAX_SERVO_POWER = 0.8;
    public static double MIN_SERVO_POWER = 0.08;
    public static double MAX_VELOCITY = 200.0; // degrees/second limit
    public static double MAX_ACCELERATION = 400.0; // degrees/second^2 limit

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
     * PID controller with velocity and acceleration limiting
     */
    private double calculatePID(double error) {
        long currentTime = System.nanoTime();
        double deltaTime = (currentTime - lastUpdateTime) / 1e9; // Convert to seconds
        lastUpdateTime = currentTime;

        if (deltaTime <= 0) deltaTime = 0.001; // Prevent divide by zero

        // Proportional
        double p = kP * error;

        // Integral with anti-windup
        integralSum += error * deltaTime;
        integralSum = Range.clip(integralSum, -100, 100);
        double i = kI * integralSum;

        // Derivative
        double derivative = (error - lastError) / deltaTime;
        double d = kD * derivative;
        lastError = error;

        // Feedforward
        double f = kF * Math.signum(error);

        double output = p + i + d + f;

        // Velocity limiting
        double targetVelocity = output * MAX_VELOCITY;
        targetVelocity = Range.clip(targetVelocity, -MAX_VELOCITY, MAX_VELOCITY);

        // Acceleration limiting
        double velocityChange = targetVelocity - lastVelocity;
        double maxVelocityChange = MAX_ACCELERATION * deltaTime;
        velocityChange = Range.clip(velocityChange, -maxVelocityChange, maxVelocityChange);
        lastVelocity += velocityChange;

        // Convert velocity to servo power (normalize by max velocity)
        output = lastVelocity / MAX_VELOCITY;

        // Apply minimum power threshold
        if (Math.abs(output) > 0.001 && Math.abs(output) < MIN_SERVO_POWER) {
            output = Math.signum(output) * MIN_SERVO_POWER;
        }

        return Range.clip(output, -MAX_SERVO_POWER, MAX_SERVO_POWER);
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

        // Calculate PID output
        double power = calculatePID(error);

        // Set servo power
        turretServo.setPower(power);
    }

    /**
     * Close alignment mode - more aggressive PID
     */
    public void closeAlign() {
        // Temporarily increase kP for close range
        double originalKp = kP;
        kP = 0.12; // Higher for close shots
        align();
        kP = originalKp;
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
            telemetry.addData("--- Vision+Encoder Turret ---", "");
            telemetry.addData("Target Visible", limelight.isTargetVisible());
            telemetry.addData("Vision TX", tx != null ? String.format("%.2f°", tx) : "N/A");
            telemetry.addData("Current Angle", String.format("%.2f°", currentAngle));
            telemetry.addData("Target Angle", String.format("%.2f°", targetAngle));
            telemetry.addData("Error", String.format("%.2f°", targetAngle - currentAngle));
            telemetry.addData("Is Aligned", isAligned());
            telemetry.addData("Is Aligning", isAligning);
            telemetry.addData("Encoder Voltage", String.format("%.3fV", turretEncoder.getVoltage()));
            telemetry.addData("Velocity", String.format("%.1f°/s", lastVelocity));
        }
    }

    /**
     * Get telemetry string for debugging
     */
    public String getTelemetry() {
        Double tx = limelight.getTx();
        return String.format(
                "Vision+Encoder Turret:\n" +
                        "  Current: %.2f°\n" +
                        "  Target: %.2f°\n" +
                        "  Error: %.2f°\n" +
                        "  Vision TX: %s\n" +
                        "  Aligned: %b\n" +
                        "  Velocity: %.1f°/s",
                currentAngle,
                targetAngle,
                targetAngle - currentAngle,
                tx != null ? String.format("%.2f°", tx) : "N/A",
                isAligned(),
                lastVelocity
        );
    }
}