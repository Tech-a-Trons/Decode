package org.firstinspires.ftc.teamcode.Season.Subsystems;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public class Midtake implements Subsystem {
    public static final Midtake INSTANCE = new Midtake();
    private Midtake() { }
    public MotorEx midtake = new MotorEx("ramp");
}
