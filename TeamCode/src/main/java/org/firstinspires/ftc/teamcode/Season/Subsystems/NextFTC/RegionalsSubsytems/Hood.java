package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems;

import com.acmerobotics.dashboard.config.Config;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;

@Config
public class Hood implements Subsystem {

    public static final Hood INSTANCE = new Hood();

    public static double closePos = 0.8;
    public static double midclosePos = 0.5;
    public static double midopenPos = 0.3;
    public static double openPos = 0.3;
    public static double BASE_POS = midopenPos;
    public static double VEL_THRESHOLD = 75;
    public static double HOOD_GAIN = 0.0003;
    public static double MAX_UP_ADJUST = 0.05;
    private final ServoEx hood;
    private boolean isOpen = false;


    public Hood() {
        hood = new ServoEx("hood");

        hood.setPosition(midopenPos);
    }

    // ===== BASIC CONTROL =====

    public void set(double pos) {
        hood.setPosition(clamp(pos));
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
    public void auto() {
        set(0.34);
        isOpen = true;
    }

    public void midclose() {
        set(midclosePos);
        isOpen = false;
    }

    public void toggle() {
        if (isOpen) close();
        else open();
    }

    // ===== VELOCITY COMPENSATION (FIXED) =====

    public void compensateFromVelocity(double targetVel, double actualVel) {

        double error = targetVel - actualVel;

        if (Math.abs(error) < VEL_THRESHOLD) {
            set(BASE_POS);
            return;
        }

        double adjust = error * HOOD_GAIN;

        adjust = Math.max(0, Math.min(MAX_UP_ADJUST, adjust));


        set(BASE_POS + adjust);
    }

    // ===== SAFETY CLAMP =====

    private double clamp(double pos) {
        return Math.max(0.0, Math.min(1.0, pos));
    }

    @Override
    public void periodic() {
        // No periodic behavior required
    }
}