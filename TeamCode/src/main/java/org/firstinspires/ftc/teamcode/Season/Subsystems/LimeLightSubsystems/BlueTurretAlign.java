//package org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import dev.nextftc.core.subsystems.Subsystem;
//import dev.nextftc.hardware.impl.ServoEx;
//
//@Config
//public class BlueTurretAlign implements Subsystem {
//
//    public static final BlueTurretAlign INSTANCE = new BlueTurretAlign();
//
//    public static double kP = 0.01;
//    public static double ALIGN_TOLERANCE_DEG = 1.0;
//
//    public static double SERVO_MIN = -1.0;
//    public static double SERVO_MAX =  1.0;
//
//    private final Servo turret;
//    private double servoPosition = 0.0;
//
//    private BlueExperimentalDistanceLExtractor limelight;
//
//    public BlueTurretAlign() {
//        turret = new ServoEx("turret").getServo();
//    }
//
//    public void setLimelight(BlueExperimentalDistanceLExtractor ll) {
//        this.limelight = ll;
//    }
//
//    private boolean isBlueTag(Integer tagId) {
//        return tagId != null && (tagId == 20 || tagId == 24);
//    }
//
//    public boolean aligned() {
//        if (limelight == null) return false;
//
//        Integer tagId = limelight.getTagId();
//        if (!isBlueTag(tagId)) return false;
//
//        Double tx = limelight.getTx();
//        if (tx == null) return false;
//
//        return Math.abs(tx) <= ALIGN_TOLERANCE_DEG;
//    }
//
//    @Override
//    public void periodic() {
//        if (limelight == null) return;
//        if (!limelight.isTargetVisible()) return;
//
//        Integer tagId = limelight.getTagId();
//        if (!isBlueTag(tagId)) return;
//
//        Double tx = limelight.getTx();
//        if (tx == null) return;
//
//        if (Math.abs(tx) <= ALIGN_TOLERANCE_DEG) return;
//
//        double delta = kP * tx;
//        servoPosition -= delta;
//
//        servoPosition = Math.max(SERVO_MIN, Math.min(SERVO_MAX, servoPosition));
//
//        turret.setPosition(0.5 + servoPosition);
//    }
//}
package org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.core.subsystems.Subsystem;

//Lines 230,231,234,237,238,240 & 241 need to be tuned
public class BlueTurretAlign implements Subsystem {

    public static final BlueTurretAlign INSTANCE = new BlueTurretAlign();

    /* ================= PID TUNING ================= */
    public static double kP = 0.2; //0.02
    public static double kD = 0.00; //0.001

    public static double ALIGN_TOLERANCE_DEG = 1.5;
    public static double MAX_SERVO_STEP = 0.03;

    /* ================= SERVO MODEL ================= */
    private static final double SERVO_MIN = 0.0;
    private static final double SERVO_MAX = 1.0;

    private static final double TURRET_MIN_DEG = -180.0;
    private static final double TURRET_MAX_DEG =  180.0;

    private static final double SERVO_CENTER = 0.5;

    /* ================= HARDWARE ==================== */
    private Servo turret;
    private BlueExperimentalDistanceLExtractor ll;

    /* ================= STATE ======================= */
    private double lastError = 0.0;
    private long lastTimeNs = 0;

    private boolean initialized = false;
    private boolean alignmentActive = false;

    public BlueTurretAlign() {}

    public void initHardware(HardwareMap hw) {
        turret = hw.get(Servo.class, "turret");
        turret.setPosition(SERVO_CENTER);
        lastTimeNs = System.nanoTime();
        initialized = true;
    }

    public void setLimelight(BlueExperimentalDistanceLExtractor ll) {
        this.ll = ll;
    }

    public void setAlignmentActive(boolean active) {
        alignmentActive = active;
        lastError = 0.0;
        lastTimeNs = System.nanoTime();
    }

    @Override
    public void periodic() {
        if (!initialized || !alignmentActive || ll == null) return;
        if (!ll.isTargetVisible()) return;

        Double tx = ll.getTx();
        if (tx == null) return;

        /* ---------- DEAD BAND ---------- */
        if (Math.abs(tx) <= ALIGN_TOLERANCE_DEG) {
            lastError = tx;
            return;
        }

        /* ---------- PID (ANGLE SPACE) ---------- */
        long now = System.nanoTime();
        double dt = (now - lastTimeNs) / 1e9;
        lastTimeNs = now;

        double error = -tx;                // desiredAngle - currentAngle
        double derivative = (error - lastError) / dt;
        lastError = error;

        double outputDeg = (kP * error) + (kD * derivative);

        /* ---------- MAP ANGLE → SERVO ---------- */
        double currentServo = turret.getPosition();
        double currentAngle = map(
                currentServo,
                SERVO_MIN, SERVO_MAX,
                TURRET_MIN_DEG, TURRET_MAX_DEG
        );

        double targetAngle = currentAngle + outputDeg;

        targetAngle = clamp(targetAngle, TURRET_MIN_DEG, TURRET_MAX_DEG);

        double targetServo = map(
                targetAngle,
                TURRET_MIN_DEG, TURRET_MAX_DEG,
                SERVO_MIN, SERVO_MAX
        );

        /* ---------- LIMIT STEP SIZE ---------- */
        double delta = clamp(
                targetServo - currentServo,
                -MAX_SERVO_STEP,
                MAX_SERVO_STEP
        );

        turret.setPosition(currentServo + delta);
    }

    /* ================= UTILS ================= */

    private static double map(double x,
                              double inMin, double inMax,
                              double outMin, double outMax) {
        return (x - inMin) * (outMax - outMin)
                / (inMax - inMin) + outMin;
    }

    private static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}
