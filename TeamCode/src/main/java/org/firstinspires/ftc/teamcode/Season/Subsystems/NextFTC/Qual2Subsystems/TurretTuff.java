package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
import com.qualcomm.robotcore.hardware.HardwareMap;


@Config
public class TurretTuff implements Subsystem {
    public static final TurretTuff INSTANCE = new TurretTuff();
    // Hardware
    private CRServo turretServo;
    private AnalogInput turretEncoder;

    // Target position in field coordinates
    public static double TARGET_X = 122.0;
    public static double TARGET_Y = 122.0;

    // PID coefficients - tune these values
    public static double kP = 0.015;
    public static double kI = 0.0001;
    public static double kD = 0.0008;
    public static double kF = 0.0; // Feedforward if needed

    // Angle limits (in degrees)
    public static double MIN_ANGLE = -180.0;
    public static double MAX_ANGLE = 70.0;

    // Gear ratio: servo is 80 tooth, turret is 200 tooth
    private static final double GEAR_RATIO = 200.0 / 80.0; // 2.5:1

    // Encoder constants
    private static final double ENCODER_VOLTAGE_RANGE = 3.3; // Axon Max outputs 0-3.3V
    private static final double DEGREES_PER_REVOLUTION = 360.0;

    // Tolerance and control
    public static double POSITION_TOLERANCE = 2.0; // degrees
    public static double MAX_SERVO_POWER = 0.6;
    public static double MIN_SERVO_POWER = 0.08; // Minimum power to overcome friction

    // PID variables
    private double integralSum = 0;
    private double lastError = 0;
    private long lastUpdateTime = 0;

    // State tracking
    private double currentAngle = 0;
    private double targetAngle = 0;
    private double encoderOffset = 0;

    public void init(HardwareMap hardwareMap) {
        // Initialize hardware
        this.turretServo = hardwareMap.get(CRServo.class, "turret");
        this.turretEncoder = hardwareMap.get(AnalogInput.class, "turret_encoder");

        // Calibrate encoder offset at startup
        calibrateEncoder();

        lastUpdateTime = System.nanoTime();
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
     * Calculate the angle to target based on robot's current pose
     */
    public double calculateTargetAngle() {
        // Get current robot pose from PedroPathing
        Pose robotPose = PedroComponent.follower().getPose();

        double robotX = robotPose.getX();
        double robotY = robotPose.getY();
        double robotHeading = Math.toDegrees(robotPose.getHeading());

        // Calculate vector from robot to target
        double deltaX = TARGET_X - robotX;
        double deltaY = TARGET_Y - robotY;

        // Calculate absolute angle to target (field-centric)
        double angleToTarget = Math.toDegrees(Math.atan2(deltaY, deltaX));

        // Convert to robot-centric angle
        double turretAngle = angleToTarget - robotHeading;

        // Normalize angle to -180 to 180
        while (turretAngle > 180) turretAngle -= 360;
        while (turretAngle < -180) turretAngle += 360;

        // Clamp to mechanical limits
        turretAngle = Range.clip(turretAngle, MIN_ANGLE, MAX_ANGLE);

        return turretAngle;
    }

    /**
     * PID controller for turret positioning
     */
    private double calculatePID(double error) {
        long currentTime = System.nanoTime();
        double deltaTime = (currentTime - lastUpdateTime) / 1e9; // Convert to seconds
        lastUpdateTime = currentTime;

        // Proportional
        double p = kP * error;

        // Integral with anti-windup
        integralSum += error * deltaTime;
        integralSum = Range.clip(integralSum, -50, 50); // Prevent integral windup
        double i = kI * integralSum;

        // Derivative
        double derivative = (error - lastError) / deltaTime;
        double d = kD * derivative;
        lastError = error;

        // Feedforward
        double f = kF * Math.signum(error);

        double output = p + i + d + f;

        // Apply minimum power threshold
        if (Math.abs(output) > 0.001 && Math.abs(output) < MIN_SERVO_POWER) {
            output = Math.signum(output) * MIN_SERVO_POWER;
        }

        return Range.clip(output, -MAX_SERVO_POWER, MAX_SERVO_POWER);
    }

    /**
     * Main update loop - call this continuously in your periodic() or loop()
     */
    public void update() {
        // Update current angle from encoder
        getCurrentAngle();

        // Calculate target angle based on odometry
        targetAngle = calculateTargetAngle();

        // Calculate error
        double error = targetAngle - currentAngle;

        // Calculate PID output
        double power = calculatePID(error);

        // Set servo power
        turretServo.setPower(power);
    }

    /**
     * Check if turret is at target position
     */
    public boolean atTarget() {
        return Math.abs(targetAngle - currentAngle) < POSITION_TOLERANCE;
    }

    /**
     * Stop the turret
     */
    public void stop() {
        turretServo.setPower(0);
        integralSum = 0;
        lastError = 0;
    }

    /**
     * Set a new target position
     */
    public static void setTarget(double x, double y) {
        TARGET_X = x;
        TARGET_Y = y;
    }

    /**
     * Get telemetry data for debugging
     */
    public String getTelemetry() {
        Pose robotPose =  PedroComponent.follower().getPose();
        return String.format(
                "Turret State:\n" +
                        "  Current: %.2f°\n" +
                        "  Target: %.2f°\n" +
                        "  Error: %.2f°\n" +
                        "  At Target: %b\n" +
                        "  Robot Pose: (%.1f, %.1f, %.1f°)\n" +
                        "  Target: (%.1f, %.1f)",
                currentAngle, targetAngle,
                targetAngle - currentAngle,
                atTarget(),
                robotPose.getX(), robotPose.getY(), Math.toDegrees(robotPose.getHeading()),
                TARGET_X, TARGET_Y
        );
    }

    /**
     * Manual control for testing
     */
    public void setManualPower(double power) {
        turretServo.setPower(Range.clip(power, -MAX_SERVO_POWER, MAX_SERVO_POWER));
    }

    /**
     * Reset PID controller
     */
    public void resetPID() {
        integralSum = 0;
        lastError = 0;
        lastUpdateTime = System.nanoTime();
    }
}