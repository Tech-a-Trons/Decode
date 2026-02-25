package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public class CompliantIntake implements Subsystem {
    public static final CompliantIntake INSTANCE = new CompliantIntake();
    public CompliantIntake() { }
    public MotorEx intake = new MotorEx("in");
    public void on() {
       intake.setPower(1);
    }
    public void repel() {
        intake.setPower(-1);
    }
    public void slight(){intake.setPower(-0.3);}
    public void mid() {
        intake.setPower(0.5);
    }
    public void off() {
        intake.setPower(0);
    }


}
