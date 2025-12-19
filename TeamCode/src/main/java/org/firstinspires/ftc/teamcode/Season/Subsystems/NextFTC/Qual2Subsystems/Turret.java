package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.CRServoEx;
import com.acmerobotics.dashboard.config.Config;

@Config
public class Turret implements Subsystem {

    public static final Turret INSTANCE = new Turret();

    public static double openTime = 0.6;
    public static double midopenTime = 0.2;
    public static double midcloseTime = 0.2;
    public static double closeTime = 0.6;

    private final CRServoEx turret;
    private final ElapsedTime timer = new ElapsedTime();

    private double targetTime = 0;
    private double currentPosition = 0;
    private boolean moving = false;
    private boolean directionForward = true;
    private boolean isOpen = false;

    public Turret() {
        turret = new CRServoEx("turret");
    }

    private void moveBy(double time, boolean forward) {
        targetTime = time;
        directionForward = forward;
        timer.reset();
        moving = true;
        turret.setPower(forward ? 1.0 : -1.0);
    }

    public void open() {
        moveBy(openTime, true);
        isOpen = true;
    }

    public void close() {
        moveBy(closeTime, false);
        isOpen = false;
    }

    public void midopen() {
        moveBy(midopenTime, true);
        isOpen = true;
    }

    public void midclose() {
        moveBy(midcloseTime, false);
        isOpen = false;
    }

    public void resetToStart() {
        if (currentPosition > 0) moveBy(currentPosition, false);
    }

    public void toggle() {
        if (isOpen) close(); else open();
    }

    @Override
    public void periodic() {
        if (moving) {
            double delta = timer.seconds();
            if (delta >= targetTime) {
                turret.setPower(0);
                currentPosition += directionForward ? targetTime : -targetTime;
                if (currentPosition < 0) currentPosition = 0;
                moving = false;
            }
        }
    }
}