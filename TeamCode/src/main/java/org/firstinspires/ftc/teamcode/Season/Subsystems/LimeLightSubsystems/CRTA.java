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
    double power;
    double error;
    double derivative;
    // Encoder constants (tune for your setup)
    private static final double VOLTAGE_PER_DEGREE = 3.3 / 360.0;  // 0-3.3V = 0-360°
    private static final double START_ANGLE_DEG = 0.0;             // your desired start angle
    private static final double ANGLE_TOLERANCE = 2.0;             // degrees

    private boolean homed = false;

    private double startingPwr = 0;

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
        homed = false;
        ll.startReading();
        tx = ll.getTx();
        lastTx = 0.0;
        startingPwr = turret.getPower();
    }

    private double targetAngleDeg = 0.0;  // Desired absolute turret angle
    private static final double KP = 0.004;  // Position gains
    private static final double KI = 0.001;
    private static final double KD = 0.12;
    private double integral = 0.0, lastError = 0.0;
    boolean atneg = false;
    boolean atpos = false;

//    public void rotate() {
////        ll.update();
////
////        if (Objects.equals(ll.getStatus(), "No data (stale)")) {
////            if (turret.getPower() != 0.35 && !atpos) {
////                applyPower(0.35);
////                atneg = true;
////            } else if (turret.getPower() == 0.35 && atneg) {
////                applyPower(startingPwr);
////                atpos = true;
////            } else {
////                applyPower(-0.35);
////            }
////            atneg = false;
////            atpos = false;
////            return;
////        }
////
//////        if (!ll.isTargetVisible()) {
//////            // No target: hold last target angle or scan gently
//////            double currentAngle = readEncoderAngleDeg();
//////            double holdError = targetAngleDeg - currentAngle;
//////            if (Math.abs(holdError) > 3.0) {  // Only correct if drifted
//////                applyPower(KP * holdError);
//////            }
//////            return;
//////        }
////
////        // Compute target angle from tx (calibrate offset once)
////        tx = ll.getTx();
////        if (tx == null || tx.isInfinite() || tx.isNaN()) {
////            tx = 0.0;
////        }
////
////        // Position PID to targetAngleDeg
////        double currentAngle = readEncoderAngleDeg();
////        targetAngleDeg = Math.max(-180, Math.min(180, currentAngle + tx));
////        error = targetAngleDeg - currentAngle;
////
////        // Normalize error to -180/+180
////        while (error > 180) error -= 360;
////        while (error < -180) error += 360;
////
////        if (Math.abs(error) <= 0.2) {  // Wider tolerance
////            turret.setPower(0);
////            integral = 0;
////            return;
////        }
////
////        integral += error * 0.02;  // dt=50ms
////        integral = Math.max(-1.0, Math.min(1.0, integral));
////        derivative = error - lastError;
////        power = KP * error + KI * integral + KD * derivative;
////
////        applyPower(power);  // Clamps -0.4 to 0.4
////
////        lastError = error;
//
//        ll.update();
//        tx = ll.getTx();
//
//        if (tx == null || !ll.isTargetVisible()) {
//            turret.setPower(0);
//            return;
//        }
//
//        double kP = 0.02;
//        double power = -kP * tx;
//
//// Deadband
//        if (Math.abs(tx) < 1.5) power = 0;
//
//        applyPower(power);
//
//    }

// ---- constants ----
private static final double KP_TX = 0.005;      // lower than before
    private static final double MAX_POWER = 0.75;   // CRServo safe
    private static final double TX_DEADBAND = 1.5;

    private static final double SCAN_POWER = 0.75;
    private static final long SCAN_TIME_MS = 1500;

    // ---- state ----
    private boolean scanning = false;
    private boolean scanPositive = true;
    private long lastScanSwitch = 0;

    public void rotate() {
        ll.update();
        Double txRaw = ll.getTx();

        boolean hasTarget =
                txRaw != null &&
                        !txRaw.isNaN() &&
                        !txRaw.isInfinite() &&
                        ll.isTargetVisible();

        // ==========================
        // SCAN MODE (NO TARGET)
        // ==========================
        if (!hasTarget) {
            long now = System.currentTimeMillis();

            if (!scanning) {
                scanning = true;
                scanPositive = true;
                lastScanSwitch = now;
            }

            if (now - lastScanSwitch > SCAN_TIME_MS) {
                scanPositive = !scanPositive;
                lastScanSwitch = now;
            }

            turret.setPower(scanPositive ? SCAN_POWER : -SCAN_POWER);
            return;
        }

        // ==========================
        // TARGET ACQUIRED
        // ==========================
        scanning = false;

        double tx = txRaw;

        // DEADZONE
        if (Math.abs(tx) < TX_DEADBAND) {
            turret.setPower(0);
            return;
        }

        // CRITICAL: inverted AND rate-limited
        double power = -KP_TX * tx;

        // Clamp
        power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));

        turret.setPower(power);
    }


    private void applyPower(double apower) {
        apower = Math.max(-1, Math.min(1, apower));
        turret.setPower(apower);
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

    public double StartingPower() {
        return startingPwr;
    }

    public double AppliedPwr() {
        return power;
    }

    public double Error() {
        return error;
    }

    public double getIntegral() {
        return integral;
    }

    public double getDerivative() {
        return derivative;
    }

    public double EncoderVoltage() {
        return axonEncoder.getVoltage();
    }
}
