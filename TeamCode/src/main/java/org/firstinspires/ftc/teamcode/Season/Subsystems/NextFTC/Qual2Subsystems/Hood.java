package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.ServoEx;
import com.acmerobotics.dashboard.config.Config;

@Config
public class Hood implements Subsystem {

    public static final Hood INSTANCE = new Hood();

    public static double openPos = 0.6;
    public static double midPos = 0.4;
    public static double closePos = 0;

    private final ServoEx hood;
    private boolean isOpen = false;

    private Hood() {
        hood = new ServoEx("hood");
    }

    public void set(double pos) {
        hood.setPosition(pos);
    }

    public void open() {
        set(openPos);
        isOpen = true;
    }

    public void close() {
        set(closePos);
        isOpen = false;
    }

    public void toggle() {
        if (isOpen) close(); else open();
    }



    @Override
    public void periodic() { }
}