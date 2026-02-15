package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;

@Config
public class Hood implements Subsystem {
    public static final Hood INSTANCE = new Hood();

    public static double closePos = 0.8;
    public static double midclosePos = 0.5;
    public static double midopenPos = 0.3;
    public static double openPos = 0.3;
    public static double complete = 0.1;
    public static double BASE_POS = midopenPos;
    public static double VEL_THRESHOLD = 75;
    public static double HOOD_GAIN = 0.0003;
    public static double MAX_UP_ADJUST = 0.05;

    private ServoEx hood = null;
    private boolean isOpen = false;
    private double targetPosition = midopenPos;

    private Hood() {
        // Don't initialize hardware in constructor
    }

    // This gets called automatically by NextFTC when the subsystem is registered
    private void ensureInitialized() {
        if (hood == null) {
            try {
                hood = new ServoEx("hood");
                hood.setPosition(targetPosition);
            } catch (Exception e) {
                // Hardware not available yet, will retry next call
            }
        }
    }

    // ===== BASIC CONTROL =====

    public void set(double pos) {
        targetPosition = clamp(pos);
        ensureInitialized();
        if (hood != null) {
            hood.setPosition(targetPosition);
        }
    }

    public void close() {
        set(closePos);
        isOpen = false;
    }

    public void midclose() {
        set(midclosePos);
        isOpen = false;
    }

    public void midopen() {
        set(midopenPos);
        isOpen = true;
    }

    public void open() {
        set(openPos);
        isOpen = true;
    }

    public void complete() {
        set(complete);
        isOpen = false;
    }

    public void auto() {
        set(0.34);
        isOpen = true;
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
        // Ensure hardware is initialized on first periodic call
        ensureInitialized();
    }


}