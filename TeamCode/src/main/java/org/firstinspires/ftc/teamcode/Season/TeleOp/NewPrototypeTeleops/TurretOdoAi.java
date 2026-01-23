package org.firstinspires.ftc.teamcode.Season.TeleOp.NewPrototypeTeleops;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.TurretOdo;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;

@Config
public class TurretOdoAi implements Subsystem {

    public static final TurretOdoAi INSTANCE = new TurretOdoAi();

    // ------------------ Hardware ------------------
    private Servo turretServo;
    private AnalogInput turretAbsoluteEncoder; // absolute encoder

    // ------------------ Robot Pose ------------------
    private double x = 0;
    private double y = 0;
    private double heading = 0;

    // ------------------ Target (Red Goal) ------------------
    public static double xr = 121;
    public static double yr = 121;

    // ------------------ Turret ------------------
    private double turretAngleDeg = 0;      // target angle
    private double turretEncoderAngleDeg = 0; // read from encoder

    // Servo safety
    public static double SERVO_MIN = 0.0;
    public static double SERVO_MAX = 1.0;

    private TurretOdoAi() {}

    // ------------------ Initialization ------------------
    public void init(HardwareMap hardwareMap) {
        turretServo = hardwareMap.get(Servo.class, "turretServo");
        turretServo.setPosition(0.0);

        turretAbsoluteEncoder = hardwareMap.get(AnalogInput.class, "turretEncoder");
    }

    // ------------------ Loop ------------------
    @Override
    public void periodic() {

        // 1️⃣ Update robot pose
        Pose currentPose = PedroComponent.follower().getPose();
        x = currentPose.getX();
        y = currentPose.getY();
        heading = currentPose.getHeading();

        // 2️⃣ Compute field-centric angle to goal
        double fieldAngleDeg = Math.toDegrees(
                Math.atan2(yr - y, xr - x)
        );

        // 3️⃣ Convert to robot-centric turret angle
        turretAngleDeg = fieldAngleDeg - heading;
        turretAngleDeg = normalizeDegrees(turretAngleDeg);

        // 4️⃣ Convert angle to servo position
        double servoPos = angleToServo(turretAngleDeg);
        turretServo.setPosition(servoPos);

        // 5️⃣ Read absolute encoder angle
        turretEncoderAngleDeg = readEncoderDegrees();
    }

    // ------------------ Helper Functions ------------------

    /**
     * Convert degrees to servo position (0.0 -> 1.0)
     */
    private double angleToServo(double angleDeg) {
        double pos = angleDeg / 360.0;
        return clamp(pos, SERVO_MIN, SERVO_MAX);
    }

    /**
     * Normalize any angle to [0, 360)
     */
    private double normalizeDegrees(double angle) {
        angle %= 360;
        if (angle < 0) angle += 360;
        return angle;
    }

    /**
     * Clamp a value to min/max
     */
    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    /**
     * Read absolute encoder (0V -> 3.3V) and convert to 0-360°
     * Assumes encoder is 0V at 0°, 3.3V at 360°
     */
    private double readEncoderDegrees() {
        double voltage = turretAbsoluteEncoder.getVoltage();
        double maxVoltage = turretAbsoluteEncoder.getMaxVoltage(); // usually 3.3V
        return (voltage / maxVoltage) * 360.0;
    }

    // ------------------ Getters ------------------
    public double getX() { return x; }
    public double getY() { return y; }
    public double getHeading() { return heading; }
    public double getTurretAngleDeg() { return turretAngleDeg; }
    public double getTurretEncoderAngleDeg() { return turretEncoderAngleDeg; }
}
