package org.firstinspires.ftc.teamcode.Season.Subsystems.Sensors;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class VoltageGet {
    private static double nominalVoltage = 12.5;//prev 12.75
    private static VoltageSensor voltageSensor;

    // Initialize once in your OpMode or Robot class
    public static void init(HardwareMap hardwareMap) {
        // Just get the first available voltage sensor (usually Control Hub)
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    // Returns the current Control Hub voltage
    public static double getVoltage() {
        if (voltageSensor == null) return nominalVoltage; // fallback if not initialized
        return voltageSensor.getVoltage();
    }

    // Returns power adjusted for current voltage
    public static double regulate(double targetPower) {
        double currentVoltage = getVoltage();
        double compensated = targetPower * (nominalVoltage / currentVoltage);
        return Math.max(-1.0, Math.min(1.0, compensated)); // clamp between -1 and 1
    }

    // regulate with custom volts
    public static double CustomRegulate(double targetPower, double customNominal) {
        double currentVoltage = getVoltage();
        double compensated = targetPower * (customNominal / currentVoltage);
        return Math.max(-1.0, Math.min(1.0, compensated));
    }
}
