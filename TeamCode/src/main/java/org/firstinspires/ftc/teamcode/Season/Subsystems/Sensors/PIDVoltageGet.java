package org.firstinspires.ftc.teamcode.Season.Subsystems.Sensors;

import com.qualcomm.robotcore.hardware.VoltageSensor;

public class PIDVoltageGet {

    private static final double NOMINAL_VOLTAGE = 12.5;
    private final VoltageSensor voltageSensor;

    public PIDVoltageGet(VoltageSensor voltageSensor) {
        this.voltageSensor = voltageSensor;
    }

    public double regulate(double power) {
        double voltage = voltageSensor.getVoltage();
        return power * (NOMINAL_VOLTAGE / voltage);
    }
}
