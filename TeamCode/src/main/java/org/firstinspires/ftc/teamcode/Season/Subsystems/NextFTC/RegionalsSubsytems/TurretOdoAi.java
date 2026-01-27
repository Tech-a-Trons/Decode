package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
//import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.AnalogInput;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;

//@Config
public class TurretOdoAi implements Subsystem {

    public static final TurretOdoAi INSTANCE = new TurretOdoAi();

    // ------------------ Hardware ------------------
//    private Servo turretServo;
    private AnalogInput turretAbsoluteEncoder;

    // ------------------ Robot Pose ------------------
    private double x = 0;
    private double y = 0;
    private double heading = 0;

    // ------------------ Target (Red Goal) ------------------
    public static double xt = 121;
    public static double yt = 121;

    // ------------------ Turret ------------------
    private double turretAngleDeg = 0;      // target angle
    private double turretEncoderAngleDeg = 0; // actual turret angle from encoder

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

        PedroComponent.follower().update(); // manually update odometry
        // 1️⃣ Update robot pose
        Pose currentPose = PedroComponent.follower().getPose();
        x = currentPose.getX();
        y = currentPose.getY();
        heading = (Math.toDegrees(currentPose.getHeading()));

        // 2️⃣ Compute field-centric angle to goal
        double fieldAngleDeg = Math.toDegrees(
                Math.atan2(yt - y, xt - x)
        );

        // 3️⃣ Convert to robot-centric turret angle
        turretAngleDeg = fieldAngleDeg - heading;
        turretAngleDeg = normalizeDegrees(turretAngleDeg);

        // 4️⃣ Read absolute encoder
//        turretEncoderAngleDeg = readEncoderDegrees();

        // 5️⃣ PID calculation (P-only)
//        double error = turretAngleDeg - turretEncoderAngleDeg;
//        error = normalizeDegrees(error); // make sure error is 0-360
//        if (error > 180) error -= 360;  // choose shortest rotation direction
//
//        double correction = kP * error;

        // 6️⃣ Convert target + correction to servo position
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
        angle %= 360;
        if (angle < 0) angle += 360;
        return angle;
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

//    private double readEncoderDegrees() {
//        double voltage = turretAbsoluteEncoder.getVoltage();
//        double maxVoltage = turretAbsoluteEncoder.getMaxVoltage(); // usually 3.3V
//        double angle = (voltage / maxVoltage) * 360.0;
//        angle -= TURRET_ZERO_OFFSET_DEG; // apply mechanical zero
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

//    public double getTurretEncoderAngleDeg() {
//        return turretEncoderAngleDeg;
//    }
}