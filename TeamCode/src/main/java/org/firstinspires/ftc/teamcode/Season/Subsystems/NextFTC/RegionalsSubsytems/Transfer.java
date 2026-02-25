package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public class Transfer implements Subsystem {
    public static final Transfer INSTANCE = new Transfer();
    public Transfer() { }
    public static MotorEx transfer = new MotorEx("transfer");


    public void on() {
        transfer.setPower(-1);
    }
    public void slight() {
        transfer.setPower(-0.6);
    }
    public void repel() {
        transfer.setPower(0.1);
    }
    public void fullreverse(){
        transfer.setPower(1);
    }
    public void nice() {
        transfer.setPower(-0.34);
    }
    public void advance() {
        transfer.setPower(-0.4);
    }
    public void off() {
        transfer.setPower(0);
    }
}
