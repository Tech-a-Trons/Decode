package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.extensions.pedro.PedroComponent;


public class NewHood implements Subsystem {

    public static final NewHood INSTANCE = new NewHood();

    // ===== DISTANCE ZONES =====
    //   < 15"  → 1.0
    //   15-80" → 0.5
    //   > 80"  → 0.6
    public static double CLOSE_THRESHOLD = 15.0;   // inches
    public static double FAR_THRESHOLD   = 80.0;   // inches

    public static double CLOSE_POS = 1.0;   // < 15"
    public static double MID_POS   = 0.5;   // 15-80"
    public static double FAR_POS   = 0.6;   // > 80"

    // ===== GOAL POSITION =====
    public static double GOAL_X = 121.0;
    public static double GOAL_Y = 121.0;

    // ===== FLYWHEEL VELOCITY COMPENSATION =====
    public static double VEL_THRESHOLD  = 75.0;    // RPM deadband
    public static double HOOD_VEL_GAIN  = 0.0003;  // adjustment per RPM of error
    public static double MAX_VEL_ADJUST = 0.08;    // max raise from low RPM
    public static double MIN_VEL_ADJUST = -0.04;   // max lower from high RPM

    // ===== OTHER PRESETS =====
    public static double BASE_POS    = MID_POS;
    public static double complete    = 0.1;
    public static double midclosePos = 0.5;
    public static double openPos     = 0.3;

    // ===== AUTO ADJUST =====
    public static boolean AUTO_ADJUST_ENABLED = false;

    // ===== STATE =====
    private ServoEx hood;
    private boolean isOpen         = false;
    private boolean manualOverride = false;
    private double  lastVelAdjust  = 0.0;

    private NewHood() {}

    // ===== INIT =====
    public void init(HardwareMap hardwareMap) {
        hood = new ServoEx("hood");
        hood.setPosition(MID_POS);
    }

    // ===== BASIC CONTROL =====

    public void set(double pos) {
        manualOverride = true;
        hood.setPosition(clamp(pos));
    }

    public void open() {
        set(openPos);
        isOpen = true;
    }

    public void close() {
        set(CLOSE_POS);
        isOpen = false;
    }

    public void midopen() {
        set(MID_POS);
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

    // ===== DISTANCE LOOKUP =====

    /**
     * Simple 3-zone distance lookup:
     *   < 15"  → 1.0
     *   15-80" → 0.5
     *   > 80"  → 0.6
     */
    public double getHoodPositionFromDistance(double distance) {
        if (distance < CLOSE_THRESHOLD) {
            return CLOSE_POS;
        } else if (distance <= FAR_THRESHOLD) {
            return MID_POS;
        } else {
            return FAR_POS;
        }
    }

    /**
     * Set hood based on current robot distance to goal
     */
    public void adjustForCurrentDistance() {
        if (PedroComponent.follower() == null) return;
        Pose pose = PedroComponent.follower().getPose();
        if (pose == null) return;

        double distance = Math.hypot(GOAL_X - pose.getX(), GOAL_Y - pose.getY());
        adjustForDistance(distance);
    }

    /**
     * Set hood based on a known distance
     */
    public void adjustForDistance(double distance) {
        manualOverride = false;
        hood.setPosition(getHoodPositionFromDistance(distance));
    }

    // ===== FLYWHEEL VELOCITY COMPENSATION =====

    /**
     * Adjust hood based on flywheel RPM error only (uses BASE_POS as starting point).
     * Under RPM → hood goes up (more arch). Over RPM → hood goes down (flatter).
     *
     * @param targetRPM  RPM your flywheel should be at
     * @param actualRPM  RPM your flywheel is actually at
     */
    public void compensateFromVelocity(double targetRPM, double actualRPM) {
        double error = targetRPM - actualRPM;

        if (Math.abs(error) < VEL_THRESHOLD) {
            lastVelAdjust = 0.0;
            hood.setPosition(clamp(BASE_POS));
            return;
        }

        double adjust = clamp(error * HOOD_VEL_GAIN, MIN_VEL_ADJUST, MAX_VEL_ADJUST);
        lastVelAdjust = adjust;
        hood.setPosition(clamp(BASE_POS + adjust));
    }

    /**
     * Distance lookup + flywheel RPM compensation combined.
     * This is the recommended method to call when shooting.
     *
     * @param distance   Current distance to goal in inches
     * @param targetRPM  RPM your flywheel should be at
     * @param actualRPM  RPM your flywheel is actually at
     */
    public void adjustForDistanceAndVelocity(double distance, double targetRPM, double actualRPM) {
        manualOverride = false;

        double basePos = getHoodPositionFromDistance(distance);

        double error = targetRPM - actualRPM;
        double velAdjust = 0.0;

        if (Math.abs(error) >= VEL_THRESHOLD) {
            velAdjust = clamp(error * HOOD_VEL_GAIN, MIN_VEL_ADJUST, MAX_VEL_ADJUST);
        }

        lastVelAdjust = velAdjust;
        hood.setPosition(clamp(basePos + velAdjust));
    }

    // ===== PERIODIC =====

    @Override
    public void periodic() {
        if (AUTO_ADJUST_ENABLED && !manualOverride) {
            adjustForCurrentDistance();
        }
    }

    // ===== HELPERS =====

    private double clamp(double pos) {
        return Math.max(0.0, Math.min(1.0, pos));
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    // ===== GETTERS =====
    public double getCurrentPosition()  { return hood.getPosition(); }
    public double getLastVelAdjust()    { return lastVelAdjust; }
    public boolean isManualOverride()   { return manualOverride; }
    public boolean isOpen()             { return isOpen; }
    public double getCurrentDistance() {
        if (PedroComponent.follower() == null) return 0.0;
        Pose pose = PedroComponent.follower().getPose();
        if (pose == null) return 0.0;
        return Math.hypot(GOAL_X - pose.getX(), GOAL_Y - pose.getY());
    }
}