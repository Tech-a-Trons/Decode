package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public class Transfer implements Subsystem {
    public static final Transfer INSTANCE = new Transfer();
    private Transfer() { }
    public static MotorEx transfer = new MotorEx("transfer");


    public void on() {
        transfer.setPower(-1);
    }
    public void repel() {
        transfer.setPower(0.2);
    }
    public void advance() {
        transfer.setPower(-0.3);
    }
    public void off() {
        transfer.setPower(0);
    }
}
