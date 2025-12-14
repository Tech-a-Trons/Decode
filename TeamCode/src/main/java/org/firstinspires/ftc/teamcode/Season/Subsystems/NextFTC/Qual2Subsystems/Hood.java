package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.ServoEx;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Season.Subsystems.Sensors.VoltageGet;

@Config
public class Hood implements Subsystem {

    public static final Hood INSTANCE = new Hood();
    public static double openPos = 0.8;
    public static double midopenPos = 0.6;
    public static double midclosePos = 0.5;
    public static double closePos = 0.3;
    VoltageGet volt = new VoltageGet();
    private final ServoEx hood;
    private boolean isOpen = false;

    public Hood() {
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

    public void midopen() {
        set(midopenPos);
        isOpen = true;
    }

    public void midclose() {
        set(midclosePos);
        isOpen = true;
    }

    public void toggle() {
        if (isOpen) close(); else open();
    }

    @Override
    public void periodic() { }
}