package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems.RedExperimentalDistanceLExtractor;

import com.pedropathing.geometry.Pose;
import dev.nextftc.extensions.pedro.PedroComponent;

@Config
public class HybridTurretAlignmentEncoder {
    private CRServo turretServo;
    private AnalogInput turretEncoder;
    private RedExperimentalDistanceLExtractor limelight;
    private Telemetry telemetry;

    // Target position (goal location)
    public static double TARGET_X = 121;
    public static double TARGET_Y = 121;

    // Encoder constants
    private static final double ENCODER_VOLTAGE_RANGE = 3.3;
    private static final double DEGREES_PER_REVOLUTION = 360.0;
    private static final double GEAR_RATIO = 200.0 / 80.0; // 2.5:1

    // Angle limits
    public static double MIN_ANGLE = -180.0;
    public static double MAX_ANGLE = 180.0;

    // Hybrid control thresholds
    public static double ODOMETRY_COARSE_THRESHOLD = 20.0; // Switch to Limelight within 20°
    public static double LIMELIGHT_FINE_THRESHOLD = 3.0;   // Limelight alignment tolerance

    // Odometry-based PID (coarse alignment)
    public static double ODO_kP = 0.025;
    public static double ODO_kI = 0.0;
    public static double ODO_kD = 0.003;
    public static double ODO_MAX_POWER = 0.6;
    public static double ODO_MIN_POWER = 0.08;

    // Limelight-based control (fine alignment)
    public static double LL_BASE_POWER = 0.03;
    public static double LL_kP = 0.006;
    public static double LL_MAX_POWER = 0.4;
    public static double LL_ALIGNMENT_THRESHOLD = 1.5;

    // State tracking
    private double currentAngle = 0;
    private double targetAngle = 0;
    private double encoderOffset = 0;

    // PID variables for odometry mode
    private double odoIntegralSum = 0;
    private double odoLastError = 0;
    private long odoLastUpdateTime = 0;

    private boolean isAligning = false;
    private String currentMode = "IDLE"; // "ODOMETRY", "LIMELIGHT", "IDLE"

    public HybridTurretAlignmentEncoder(HardwareMap hardwareMap, RedExperimentalDistanceLExtractor limelight) {
        this.turretServo = hardwareMap.get(CRServo.class, "turret");
        this.turretEncoder = hardwareMap.get(AnalogInput.class, "turret_encoder");
        this.limelight = limelight;

        calibrateEncoder();
        odoLastUpdateTime = System.nanoTime();
    }

    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    /**
     * Calibrate encoder - set current position as reference
     */
    public void calibrateEncoder() {
        encoderOffset = getRawEncoderAngle();
    }

    /**
     * Get raw encoder angle from analog voltage
     */
    private double getRawEncoderAngle() {
        double voltage = turretEncoder.getVoltage();
        return (voltage / ENCODER_VOLTAGE_RANGE) * DEGREES_PER_REVOLUTION;
    }

    /**
     * Get current turret angle accounting for gear ratio and offset
     */
    public double getCurrentAngle() {
        double rawAngle = getRawEncoderAngle();

        double deltaAngle = rawAngle - encoderOffset;
        if (deltaAngle > 180) deltaAngle -= 360;
        if (deltaAngle < -180) deltaAngle += 360;

        currentAngle = deltaAngle * GEAR_RATIO;
        return currentAngle;
    }

    /**
     * Calculate target angle to goal using odometry
     */
    private double calculateOdometryTargetAngle() {
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
     * PID control for odometry-based alignment
     */
    private double calculateOdometryPID(double error) {
        long currentTime = System.nanoTime();
        double deltaTime = (currentTime - odoLastUpdateTime) / 1e9;
        odoLastUpdateTime = currentTime;

        if (deltaTime <= 0) deltaTime = 0.001;

        // Proportional
        double p = ODO_kP * error;

        // Integral with anti-windup
        odoIntegralSum += error * deltaTime;
        odoIntegralSum = Range.clip(odoIntegralSum, -50, 50);
        double i = ODO_kI * odoIntegralSum;

        // Derivative
        double derivative = (error - odoLastError) / deltaTime;
        double d = ODO_kD * derivative;
        odoLastError = error;

        double output = p + i + d;

        // Apply minimum power threshold
        if (Math.abs(output) > 0.001 && Math.abs(output) < ODO_MIN_POWER) {
            output = Math.signum(output) * ODO_MIN_POWER;
        }

        return Range.clip(output, -ODO_MAX_POWER, ODO_MAX_POWER);
    }

    /**
     * Limelight-based fine alignment (same as SimpleLL)
     */
    private double calculateLimelightPower() {
        Double tx = limelight.getTx();

        if (tx == null) {
            return 0.0;
        }

        // If within threshold, stop
        if (Math.abs(tx) <= LL_ALIGNMENT_THRESHOLD) {
            return 0.0;
        }

        // Calculate power based on error
        double power = LL_BASE_POWER + (LL_kP * Math.abs(tx));
        power = Math.min(power, LL_MAX_POWER);

        // Determine direction
        if (tx > LL_ALIGNMENT_THRESHOLD) {
            return -power; // Turn LEFT
        } else if (tx < -LL_ALIGNMENT_THRESHOLD) {
            return power;  // Turn RIGHT
        }

        return 0.0;
    }

    /**
     * Main hybrid alignment method
     */
    public void align() {
        getCurrentAngle();

        // Calculate odometry-based target angle
        double odometryTarget = calculateOdometryTargetAngle();
        double odometryError = odometryTarget - currentAngle;

        // Check if Limelight can see target
        boolean limelightHasTarget = limelight.getTx() != null && limelight.isTargetVisible();

        // DECISION LOGIC: Choose mode based on error and Limelight availability
        if (Math.abs(odometryError) > ODOMETRY_COARSE_THRESHOLD || !limelightHasTarget) {
            // ODOMETRY MODE: Coarse alignment using encoder + odometry
            currentMode = "ODOMETRY";
            targetAngle = odometryTarget;

            double power = calculateOdometryPID(odometryError);
            turretServo.setPower(power);
            isAligning = true;

        } else {
            // LIMELIGHT MODE: Fine alignment using vision
            currentMode = "LIMELIGHT";

            double power = calculateLimelightPower();

            if (power == 0.0) {
                // Aligned!
                stopTurret();
            } else {
                turretServo.setPower(power);
                isAligning = true;
            }
        }
    }

    /**
     * Close alignment mode
     */
    public void closeAlign() {
        // Use lower power for close shots
        double originalLLMax = LL_MAX_POWER;
        double originalOdoMax = ODO_MAX_POWER;

        LL_MAX_POWER = 0.3;
        ODO_MAX_POWER = 0.4;

        align();

        LL_MAX_POWER = originalLLMax;
        ODO_MAX_POWER = originalOdoMax;
    }

    /**
     * Far alignment mode
     */
    public void farAlign() {
        align();
    }

    /**
     * Stop turret
     */
    public void stopTurret() {
        turretServo.setPower(0);
        isAligning = false;
        currentMode = "IDLE";
    }

    /**
     * Reset PID
     */
    public void resetPID() {
        odoIntegralSum = 0;
        odoLastError = 0;
        odoLastUpdateTime = System.nanoTime();
    }

    /**
     * Check if aligned
     */
    public boolean isAligned() {
        if (currentMode.equals("LIMELIGHT")) {
            Double tx = limelight.getTx();
            return tx != null && Math.abs(tx) <= LL_ALIGNMENT_THRESHOLD;
        } else if (currentMode.equals("ODOMETRY")) {
            double error = targetAngle - currentAngle;
            return Math.abs(error) <= ODOMETRY_COARSE_THRESHOLD;
        }
        return false;
    }

    /**
     * Check if aligning
     */
    public boolean isAligning() {
        return isAligning;
    }

    /**
     * Set target position
     */
    public void setTarget(double x, double y) {
        TARGET_X = x;
        TARGET_Y = y;
    }

    /**
     * Update telemetry
     */
    public void updateTelemetry() {
        if (telemetry != null) {
            Double tx = limelight.getTx();
            Pose pose = PedroComponent.follower().getPose();
            double odometryTarget = calculateOdometryTargetAngle();
            double odometryError = odometryTarget - currentAngle;

            telemetry.addData("--- Hybrid Turret ---", "");
            telemetry.addData("Mode", currentMode);
            telemetry.addData("Current Angle", String.format("%.2f°", currentAngle));
            telemetry.addData("Odometry Target", String.format("%.2f°", odometryTarget));
            telemetry.addData("Odometry Error", String.format("%.2f°", odometryError));
            telemetry.addData("LL Target Visible", limelight.isTargetVisible());
            telemetry.addData("LL TX", tx != null ? String.format("%.2f°", tx) : "N/A");
            telemetry.addData("Is Aligned", isAligned());
            telemetry.addData("Robot Pose", String.format("(%.1f, %.1f, %.1f°)",
                    pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading())));
        }
    }

    /**
     * Get telemetry string
     */
    public String getTelemetry() {
        Double tx = limelight.getTx();
        double odometryTarget = calculateOdometryTargetAngle();
        double odometryError = odometryTarget - currentAngle;

        return String.format(
                "Hybrid Turret:\n" +
                        "  Mode: %s\n" +
                        "  Current: %.2f°\n" +
                        "  Odo Target: %.2f°\n" +
                        "  Odo Error: %.2f°\n" +
                        "  LL TX: %s\n" +
                        "  Aligned: %b",
                currentMode,
                currentAngle,
                odometryTarget,
                odometryError,
                tx != null ? String.format("%.2f°", tx) : "N/A",
                isAligned()
        );
    }
}