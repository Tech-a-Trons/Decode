package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual1Subsystem;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public class Midtake implements Subsystem {
    public static final Midtake INSTANCE = new Midtake();
    private Midtake() { }
    public static MotorEx newtake = new MotorEx("ramp");
}
