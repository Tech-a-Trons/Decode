package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ContinuousAnalogEncoder {

    // Hardware
    private final CRServo turret;
    private final AnalogInput encoder;

    // Encoder config
    private static final double MAX_VOLTAGE = 3.3; // or 5.0
    private static final double WRAP_THRESHOLD = 0.5;

    // Gear ratio: turret / servo
    private static final double GEAR_RATIO = 80.0 / 200.0; // 0.4

    // Encoder state (SERVO side)
    private double lastRawPos;
    private double continuousPos;

    // Turret offset (front-facing = 0°)
    private static final double OFFSET_DEGREES = 75.0;

    public ContinuousAnalogEncoder(HardwareMap hardwareMap) {
        turret = hardwareMap.get(CRServo.class, "turret");
        encoder = hardwareMap.get(AnalogInput.class, "turret_encoder");

        lastRawPos = getRawPosition();
        continuousPos = 0.0;
    }

    /* ------------------ SERVO CONTROL ------------------ */

    public void setPower(double power) {
        turret.setPower(power);
    }

    public void stop() {
        turret.setPower(0);
    }

    /* ------------------ ENCODER ------------------ */

    private double getRawPosition() {
        return encoder.getVoltage() / MAX_VOLTAGE; // 0.0 → 1.0 (servo)
    }

    /** Call once per loop */
    public void update() {
        double raw = getRawPosition();
        double delta = raw - lastRawPos;

        // Wrap detection
        if (delta > WRAP_THRESHOLD) {
            delta -= 1.0;
        } else if (delta < -WRAP_THRESHOLD) {
            delta += 1.0;
        }

        // Accumulate SERVO rotation
        continuousPos += delta;
        lastRawPos = raw;
    }

    /** Continuous TURRET angle in turns */
    public double getAngleTurns() {
        return continuousPos * GEAR_RATIO;
    }

    /** Continuous TURRET angle in degrees */
    public double getAngleDegrees() {
        double degrees = continuousPos * 360.0 * GEAR_RATIO;

        // Apply front offset and wrap 0–360
        degrees = degrees - OFFSET_DEGREES;
        degrees = (degrees + 360.0) % 360.0;

        return degrees;
    }

    /** Raw encoder position (servo side, 0–1) */
    public double getRawEncoder() {
        return lastRawPos;
    }

    /** Zero turret when facing forward */
    public void zero() {
        lastRawPos = getRawPosition();
        continuousPos = 0.0;
    }
}