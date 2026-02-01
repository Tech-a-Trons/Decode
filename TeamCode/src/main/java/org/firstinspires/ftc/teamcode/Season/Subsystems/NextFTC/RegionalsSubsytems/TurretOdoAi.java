package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.AnalogInput;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;

public class TurretOdoAi implements Subsystem {

    public static final TurretOdoAi INSTANCE = new TurretOdoAi();

    // ------------------ Hardware ------------------
//    private Servo turretServo;
//    private AnalogInput turretAbsoluteEncoder;

    // ------------------ Robot Pose ------------------
    private double x = 0;
    private double y = 0;
    private double heading = 0;

    // ------------------ Target (Red Goal) ------------------
    public static double xt = 60;
    public static double yt = 60;

    // ------------------ Turret ------------------
    private double turretAngleDeg = 0;      // target angle
    private double turretEncoderAngleDeg = 0; // actual turret angle from encoder
    private double distanceToTarget = 0;     // distance to target

    // Servo safety
    public static double SERVO_MIN = 0.0;
    public static double SERVO_MAX = 1.0;

    // PID constants (tune these)
    public static double kP = 0.01; // proportional gain

    // Encoder offset (tune this)
    public static double TURRET_ZERO_OFFSET_DEG = 0.0;

    private TurretOdoAi() {
    }

    // ------------------ Initialization ------------------
    public void init(HardwareMap hardwareMap) {
//        turretServo = hardwareMap.get(Servo.class, "turretServo");
//        turretServo.setPosition(0.0);
//        turretAbsoluteEncoder = hardwareMap.get(AnalogInput.class, "turretEncoder");
    }

    // ------------------ Loop ------------------
    @Override
    public void periodic() {
        // Safety check: ensure follower is initialized
        if (PedroComponent.follower() == null) {
            return;
        }

        PedroComponent.follower().update(); // manually update odometry

        // 1️⃣ Update robot pose
        Pose currentPose = PedroComponent.follower().getPose();

        // Safety check: ensure pose is not null
        if (currentPose == null) {
            return;
        }

        x = currentPose.getX();
        y = currentPose.getY();

        double headingRad = currentPose.getHeading();
        heading = Math.toDegrees(headingRad);  // Convert to degrees FIRST
        heading = (heading + 360) % 360;       // Normalize to 0-360

        // 2️⃣ Compute field-centric angle to goal
        double fieldAngleDeg = Math.toDegrees(Math.atan2(yt - y, xt - x));
        fieldAngleDeg = (fieldAngleDeg + 360) % 360; // Normalize to 0-360

        // 3️⃣ Calculate distance to target
        distanceToTarget = Math.hypot(xt - x, yt - y);

        // 4️⃣ Convert to robot-centric turret angle
        turretAngleDeg = fieldAngleDeg - heading;
        turretAngleDeg = normalizeDegrees(turretAngleDeg);

        // 5️⃣ Read absolute encoder
//        turretEncoderAngleDeg = readEncoderDegrees();

        // 6️⃣ PID calculation (P-only)
//        double error = turretAngleDeg - turretEncoderAngleDeg;
//        error = normalizeDegrees(error);
//        if (error > 180) error -= 360;
//
//        double correction = kP * error;

        // 7️⃣ Convert target + correction to servo position
//        double servoPos = angleToServo(turretAngleDeg) + correction;
//        servoPos = clamp(servoPos, SERVO_MIN, SERVO_MAX);
//        turretServo.setPosition(servoPos);
    }

    // ------------------ Helper Functions ------------------

    private double angleToServo(double angleDeg) {
        double pos = angleDeg / 360.0;
        return clamp(pos, SERVO_MIN, SERVO_MAX);
    }

    private double normalizeDegrees(double angle) {
        angle = (angle + 360) % 360;
        return angle;
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

//    private double readEncoderDegrees() {
//        double voltage = turretAbsoluteEncoder.getVoltage();
//        double maxVoltage = turretAbsoluteEncoder.getMaxVoltage();
//        double angle = (voltage / maxVoltage) * 360.0;
//        angle -= TURRET_ZERO_OFFSET_DEG;
//        return normalizeDegrees(angle);
//    }

    // ------------------ Getters ------------------
    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeading() {
        return heading;
    }

    public double getTurretAngleDeg() {
        return turretAngleDeg;
    }

    public double getDistanceToTarget() {
        return distanceToTarget;
    }

//    public double getTurretEncoderAngleDeg() {
//        return turretEncoderAngleDeg;
//    }
}