package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems;

import com.acmerobotics.dashboard.config.Config;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;

@Config
public class ManualTurret implements Subsystem {
    public static final ManualTurret INSTANCE = new ManualTurret();

    // Position presets
    public static double pos0 = 0.0;
    public static double pos30 = 0.3;
    public static double pos50 = 0.5;
    public static double pos70 = 0.7;
    public static double pos100 = 1.0;

    private ServoEx turretServo1 = null;
    private ServoEx turretServo2 = null;
    private double targetPosition = pos50; // Default to middle position

    private ManualTurret() {
        // Don't initialize hardware in constructor
    }

    // This gets called automatically by NextFTC when the subsystem is registered
    private void ensureInitialized() {
        if (turretServo1 == null || turretServo2 == null) {
            try {
                turretServo1 = new ServoEx("turretServo1");
                turretServo2 = new ServoEx("turretServo2");

                // Set both servos to target position
                turretServo1.setPosition(targetPosition);
                turretServo2.setPosition(targetPosition);
            } catch (Exception e) {
                // Hardware not available yet, will retry next call
            }
        }
    }

    // ===== BASIC CONTROL =====

    public void set(double pos) {
        targetPosition = clamp(pos);
        ensureInitialized();
        if (turretServo1 != null && turretServo2 != null) {
            turretServo1.setPosition(targetPosition);
            turretServo2.setPosition(targetPosition);
        }
    }

    // Preset positions
    public void setPos0() {
        set(pos0);
    }

    public void setPos30() {
        set(pos30);
    }

    public void setPos50() {
        set(pos50);
    }

    public void setPos70() {
        set(pos70);
    }

    public void setPos100() {
        set(pos100);
    }

    // Incremental control
    public void increment(double delta) {
        set(targetPosition + delta);
    }

    public void incrementSmall() {
        increment(0.05);
    }

    public void decrementSmall() {
        increment(-0.05);
    }

    public void incrementLarge() {
        increment(0.1);
    }

    public void decrementLarge() {
        increment(-0.1);
    }

    // ===== GETTERS =====

    public double getCurrentPosition() {
        if (turretServo1 != null) {
            return turretServo1.getPosition();
        }
        return targetPosition;
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public boolean isInitialized() {
        return turretServo1 != null && turretServo2 != null;
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