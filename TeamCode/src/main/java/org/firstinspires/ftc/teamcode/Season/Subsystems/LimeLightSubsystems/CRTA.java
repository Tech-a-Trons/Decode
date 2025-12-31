package org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;

import dev.nextftc.core.subsystems.Subsystem;

public class CRTA implements Subsystem {
    public static final CRTA INSTANCE = new CRTA();

    private CRServo turret;
    private AnalogInput axonEncoder;  // Axon encoder on analog port
    private RedExperimentalDistanceLExtractor ll;
    private Double tx;
    private Double lastTx;

    // Encoder constants (tune for your setup)
    private static final double VOLTAGE_PER_DEGREE = 3.3 / 360.0;  // 0-3.3V = 0-360°
    private static final double START_ANGLE_DEG = 0.0;             // your desired start angle
    private static final double ANGLE_TOLERANCE = 2.0;             // degrees

    private boolean homed = false;


    // Tuning constants
    private static final double TX_TOLERANCE = 2.5;  // Wider deadband

    // Optional: small recenter behavior if target lost
//    private static final double RECENTER_POWER = 0.12; // slow, gentle recenter

    public void initHardware(HardwareMap hw) {
        turret = hw.get(CRServo.class, "turret");
        ll = new RedExperimentalDistanceLExtractor(hw);
        axonEncoder = hw.get(AnalogInput.class, "turret_encoder");  // config as Analog Input
        // For a CR servo there is no "position"; ensure it is stopped at init
//        turret.setPower(0.2);
        tx = ll.getTx();
        homed = false;
        ll.startReading();
        lastTx = 0.0;
    }

    private double targetAngleDeg = 0.0;  // Desired absolute turret angle
    private static final double KP = 0.04;  // Position gains
    private static final double KI = 0.001;
    private static final double KD = 0.12;
    private double integral = 0.0, lastError = 0.0;

    public void rotate() {
        ll.update();

        if (Objects.equals(ll.getStatus(), "No data (stale)")) {
            turret.setPower(-0.75);
            sleep(10);
            turret.setPower(0.75);
            return;
        }

        if (!ll.isTargetVisible()) {
            // No target: hold last target angle or scan gently
            double currentAngle = readEncoderAngleDeg();
            double holdError = targetAngleDeg - currentAngle;
            if (Math.abs(holdError) > 3.0) {  // Only correct if drifted
                applyPower(KP * holdError);
            }
            return;
        }

        // Compute target angle from tx (calibrate offset once)
        tx = ll.getTx();
        if (tx == null || tx.isInfinite() || tx.isNaN()) {
            tx = 0.0;
        }
        targetAngleDeg = readEncoderAngleDeg() + tx;  // tx=0 → current encoder angle

        // Position PID to targetAngleDeg
        double currentAngle = readEncoderAngleDeg();
        double error = targetAngleDeg - currentAngle;

        // Normalize error to -180/+180
        while (error > 180) error -= 360;
        while (error < -180) error += 360;

        if (Math.abs(error) <= 2.0) {  // Wider tolerance
            turret.setPower(0);
            integral = 0;
            return;
        }

        integral += error * 0.02;  // dt=50ms
        integral = Math.max(-1.0, Math.min(1.0, integral));
        double derivative = error - lastError;
        double power = KP * error + KI * integral + KD * derivative;

//        applyPower(power);  // Clamps -0.4 to 0.4
//        lastError = error;
    }

    private void applyPower(double power) {
        power = Math.max(-0.75, Math.min(0.75, power));
        turret.setPower(power);
    }


    private static final double START_VOLTAGE = 2.8;  // Example: tune to your physical start angle
    private static final double VOLTAGE_RANGE = 3.3;   // Full 0-3.3V = 360°

    private double readEncoderAngleDeg() {
        double voltage = axonEncoder.getVoltage();
        // Relative angle from your calibrated START_VOLTAGE (0° at start position)
        double relativeVoltage = voltage - START_VOLTAGE;

        // Normalize to -180° to +180° range
        double angleDeg = (relativeVoltage / VOLTAGE_RANGE) * 360.0;
        while (angleDeg > 180) angleDeg -= 360;
        while (angleDeg < -180) angleDeg += 360;

        return angleDeg;
    }

    public void periodic() {}

    public double ServoPower() {
        return turret.getPower();
    }

    public double EncoderVoltage() {
        return axonEncoder.getVoltage();
    }
}
