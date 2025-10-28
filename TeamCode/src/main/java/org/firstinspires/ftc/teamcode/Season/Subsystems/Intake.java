package org.firstinspires.ftc.teamcode.Season.Subsystems;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public class Intake implements Subsystem {
    public static final Intake INSTANCE = new Intake();
    private Intake() { }
    public MotorEx activeintake = new MotorEx("activeintake");

}
