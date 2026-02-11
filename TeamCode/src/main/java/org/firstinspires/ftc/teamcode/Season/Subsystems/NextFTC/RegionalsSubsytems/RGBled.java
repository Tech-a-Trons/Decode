package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;

public class RGBled implements Subsystem {
    public static final RGBled INSTANCE = new RGBled();

    public static double violet = 0.7222;
    public static double red = 0.277;
    public static double yellow = 0.388;
    public static double green = 0.5;
    public static double BASE_POS = violet;
    public static double VEL_THRESHOLD = 75;
    public static double HOOD_GAIN = 0.0003;
    public static double MAX_UP_ADJUST = 0.05;
    private final ServoEx RGB;
    private boolean isOpen = false;


    public RGBled() {
        RGB = new ServoEx("rgbled");

    }

    // ===== BASIC CONTROL =====

    private void set(double pos) {
        RGB.setPosition(clamp(pos));
    }

    public void setViolet() {
        set(violet);
        isOpen = true;
    }

    public void setRed() {
        set(red);
        isOpen = false;
    }

    public void setYellow() {
        set(yellow);
        isOpen = true;
    }

    public void setGreen() {
        set(green);
        isOpen = false;
    }



    // ===== VELOCITY COMPENSATION (FIXED) =====



    // ===== SAFETY CLAMP =====

    private double clamp(double pos) {
        return Math.max(0.0, Math.min(1.0, pos));
    }

    @Override
    public void periodic() {
        // No periodic behavior required
    }
}

