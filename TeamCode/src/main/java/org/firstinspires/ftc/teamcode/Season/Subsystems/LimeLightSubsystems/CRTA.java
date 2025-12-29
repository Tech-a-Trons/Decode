package org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Base64;

import dev.nextftc.core.subsystems.Subsystem;

public class CRTA implements Subsystem {
    public static final CRTA INSTANCE = new CRTA();

    private CRServo turret;
    private AnalogInput axonEncoder;  // Axon encoder on analog port
    private RedExperimentalDistanceLExtractor ll;
    private Double tx;

    // Encoder constants (tune for your setup)
    private static final double VOLTAGE_PER_DEGREE = 3.3 / 360.0;  // 0-3.3V = 0-360°
    private static final double START_ANGLE_DEG = 0.0;             // your desired start angle
    private static final double ANGLE_TOLERANCE = 2.0;             // degrees

    private boolean homed = false;


    // Tuning constants
    private static final double TX_TOLERANCE = 0.5; // degrees "good enough"
    private static final double KP = 0.01;          // tune for CR servo power
    private static final double MAX_POWER = 0.6;    // max power magnitude
    private static final double MIN_POWER = 0.1;   // minimum to overcome friction

    // Optional: small recenter behavior if target lost
//    private static final double RECENTER_POWER = 0.12; // slow, gentle recenter

    public void initHardware(HardwareMap hw) {
        turret = hw.get(CRServo.class, "turret");
        ll = new RedExperimentalDistanceLExtractor(hw);
        axonEncoder = hw.get(AnalogInput.class, "turretEncoder");  // config as Analog Input
        // For a CR servo there is no "position"; ensure it is stopped at init
        turret.setPower(0.0);
        tx = ll.getTx();
        homed = false;
        ll.startReading();
    }

    public void rotate() {

        if (turret == null || ll == null) return;

        ll.update();

        // --- SCAN TO START POSITION when no target ---
        // In rotate(), when no target:
        if (!ll.isTargetVisible()) {
            double currentAngle = readEncoderAngleDeg();  // 0° = your physical start position

            double angleError = 0.0 - currentAngle;  // Always return to 0° (start)

            if (Math.abs(angleError) <= ANGLE_TOLERANCE) {
                turret.setPower(0.0);
                return;
            }

            double power = 0.015 * angleError;
            power = Math.max(-0.25, Math.min(0.25, power));
            turret.setPower(power);
        }


        tx = ll.getTx();
        if (tx == null || Double.isNaN(tx) || Double.isInfinite(tx)) {
            turret.setPower(0.0);
            return;
        }

        double error = tx;

        // Deadband around center
        if (Math.abs(error) <= TX_TOLERANCE) {
            turret.setPower(0.0);
            return;
        }

        // P-control -> power
        double power = KP * error;

        // Enforce minimum and maximum power so it actually moves but not too fast
        double sign = Math.signum(power);
        power = Math.abs(power);
        if (power > MAX_POWER) power = MAX_POWER;
        if (power < MIN_POWER) power = MIN_POWER;
        power *= sign;

        // For CRServo: -1 = full one way, +1 = full other way, 0 = stop
        turret.setPower(power);
    }

    private static final double START_VOLTAGE = 1.65;  // Example: tune to your physical start angle
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
}
