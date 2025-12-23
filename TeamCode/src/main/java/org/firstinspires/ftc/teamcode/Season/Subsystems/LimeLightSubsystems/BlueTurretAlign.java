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