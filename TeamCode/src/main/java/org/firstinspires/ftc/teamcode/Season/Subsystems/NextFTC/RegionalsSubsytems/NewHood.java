package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.extensions.pedro.PedroComponent;

@Config
public class NewHood implements Subsystem {

    public static final NewHood INSTANCE = new NewHood();

    // Position presets
    public static double closePos = 0.8;      // For close shots
    public static double midclosePos = 0.5;
    public static double midopenPos = 0.32;    // For mid-range (around 96,96)
    public static double openPos = 0.3;
    public static double complete = 0.1;
    public static double BASE_POS = midopenPos;
    public static double VEL_THRESHOLD = 75;
    public static double HOOD_GAIN = 0.0003;
    public static double MAX_UP_ADJUST = 0.05;

    // ===== DISTANCE-BASED AUTO-ADJUSTMENT SETTINGS =====
    // Target goal position (red basket for red alliance)
    public static double GOAL_X = 121.0;  // Adjust based on which goal you're shooting at
    public static double GOAL_Y = 121.0;

    // Reference point where you use midopenPos
    public static double REFERENCE_DISTANCE = 33.94;  // Distance from (96,96) to (121,121) = sqrt((121-96)^2 + (121-96)^2)

    // Distance thresholds for smooth gradient
    public static double CLOSE_DISTANCE = 25.0;    // Closer than this = closePos
    public static double FAR_DISTANCE = 40.0;      // Farther than this = midopenPos

    // Auto-adjust mode
    public static boolean AUTO_ADJUST_ENABLED = false;

    private ServoEx hood;
    private boolean isOpen = false;
    private boolean manualOverride = false;
    public void init(HardwareMap hardwareMap) {
        hood = new ServoEx("hood");
        hood.setPosition(midopenPos);
    }


    // ===== BASIC CONTROL =====

    public void set(double pos) {
        manualOverride = true;  // Manual control disables auto-adjust
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

    public void complete() {
        set(complete);
        isOpen = false;
    }

    public void midclose() {
        set(midclosePos);
        isOpen = false;
    }

    public void toggle() {
        if (isOpen) close();
        else open();
    }

    // ===== DISTANCE-BASED CALCULATION (like TurretPID) =====

    /**
     * Calculate hood position based on distance to goal
     * Similar to how TurretPID calculates velocity from distance
     */
    public double getHoodPositionFromDistance(double distance) {
        // Closer than CLOSE_DISTANCE → use closePos (high hood angle for close shots)
        if (distance <= CLOSE_DISTANCE) {
            return closePos;
        }

        // Farther than FAR_DISTANCE → use midopenPos (lower hood angle for far shots)
        if (distance >= FAR_DISTANCE) {
            return midopenPos;
        }

        // In between → smooth linear interpolation (gradient)
        // As distance increases, hood position decreases (from close to open)
        double ratio = (distance - CLOSE_DISTANCE) / (FAR_DISTANCE - CLOSE_DISTANCE);
        double hoodPos = closePos - (ratio * (closePos - midopenPos));

        return clamp(hoodPos);
    }

    /**
     * Set hood based on current distance to goal (call this when shooting)
     */
    public void adjustForCurrentDistance() {
        if (PedroComponent.follower() == null) return;

        Pose pose = PedroComponent.follower().getPose();
        if (pose == null) return;

        double distance = Math.hypot(
                GOAL_X - pose.getX(),
                GOAL_Y - pose.getY()
        );

        adjustForDistance(distance);
    }

    /**
     * Set hood based on a specific distance
     */
    public void adjustForDistance(double distance) {
        manualOverride = false;  // Re-enable auto mode
        double pos = getHoodPositionFromDistance(distance);
        hood.setPosition(pos);
    }

    // ===== VELOCITY COMPENSATION (FIXED) =====

    public void compensateFromVelocity(double targetVel, double actualVel) {
        manualOverride = true;  // Velocity compensation disables auto-adjust

        double error = targetVel - actualVel;

        if (Math.abs(error) < VEL_THRESHOLD) {
            hood.setPosition(clamp(BASE_POS));
            return;
        }

        double adjust = error * HOOD_GAIN;
        adjust = Math.max(0, Math.min(MAX_UP_ADJUST, adjust));

        hood.setPosition(clamp(BASE_POS + adjust));
    }

    // ===== SAFETY CLAMP =====

    private double clamp(double pos) {
        return Math.max(0.0, Math.min(1.0, pos));
    }

    public double getCurrentPosition() {
        return hood.getPosition();
    }

    public double getCurrentDistance() {
        if (PedroComponent.follower() == null) return 0.0;
        Pose pose = PedroComponent.follower().getPose();
        if (pose == null) return 0.0;

        return Math.hypot(
                GOAL_X - pose.getX(),
                GOAL_Y - pose.getY()
        );
    }

    @Override
    public void periodic() {
        // Continuously auto-adjust if enabled and not manually overridden
        if (AUTO_ADJUST_ENABLED && !manualOverride) {
            adjustForCurrentDistance();
        }
    }


}