package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual1Subsystem;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.impl.MotorEx;

public class OuttakeSetPower implements Subsystem {
    public static final OuttakeSetPower INSTANCE = new OuttakeSetPower();
    private OuttakeSetPower() { }
    public static MotorGroup outtake = new MotorGroup(
            new MotorEx("outtake1").reversed(),
            new MotorEx("outtake2")
    );
}
