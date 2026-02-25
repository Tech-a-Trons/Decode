package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterPID {

    public static ShooterPID INSTANCE;

    // ── Tuning ────────────────────────────────────────────────────────────
    public static double kP = 0.012;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double VELO_TOL = 75;

    // Always store positive target — motors run negative internally
    public static double activeTargetVelocity = 0;

    // ── Hardware ──────────────────────────────────────────────────────────
    private DcMotorEx outtakeLeft;
    private DcMotorEx outtakeRight;

    // ── PID state ─────────────────────────────────────────────────────────
    private double integralSum = 0;
    private double lastError   = 0;
    private long   lastTime    = 0;

    // ── Private constructor — call init() from OpMode ─────────────────────
    private ShooterPID(HardwareMap hardwareMap) {
        outtakeLeft  = hardwareMap.get(DcMotorEx.class, "outtakeleft");
        outtakeRight = hardwareMap.get(DcMotorEx.class, "outtakeright");

        // Match AutoOuttake: outtakeright is reversed
        outtakeLeft.setDirection(DcMotor.Direction.FORWARD);
        outtakeRight.setDirection(DcMotor.Direction.REVERSE);

        outtakeLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        outtakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public static void init(HardwareMap hardwareMap) {
        INSTANCE = new ShooterPID(hardwareMap);
    }

    // ── Call every loop ───────────────────────────────────────────────────
    public void update() {
        double current = getActualVelocity(); // negative due to motor direction
        double target  = -activeTargetVelocity; // negate to match motor direction
        double error   = target - current;

        long   now = System.nanoTime();
        double dt  = (lastTime == 0) ? 0.02 : (now - lastTime) / 1e9;
        lastTime   = now;

        integralSum += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        double output = kP * error + kI * integralSum + kD * derivative;
        output = Math.max(-1.0, Math.min(1.0, output));

        outtakeLeft.setPower(output);
        outtakeRight.setPower(output);
    }

    // ── Speed setters ─────────────────────────────────────────────────────
    public void setTargetVelocity(double velocity) {
        activeTargetVelocity = velocity; // store positive
        resetPID();
    }

    public void setShooterFromDistance(double distance) {
        double v = 0.041 * distance * distance - 2.9 * distance + 1250;
        if (distance > 95) v += -(0.25 * (distance - 42.5)) + 60;
        v = Math.max(1200, Math.min(2000, v));
        setTargetVelocity(v);
    }

    public void stop() {
        activeTargetVelocity = 0;
        resetPID();
        outtakeLeft.setPower(0);
        outtakeRight.setPower(0);
    }

    // ── Getters ───────────────────────────────────────────────────────────
    // Returns raw velocity (negative due to motor direction)
    public double getActualVelocity() {
        return (outtakeLeft.getVelocity() + outtakeRight.getVelocity()) / 2.0;
    }

    // Returns absolute velocity for comparisons
    public double getAbsVelocity() {
        return Math.abs(getActualVelocity());
    }

    public boolean atTargetVelocity() {
        return Math.abs(getAbsVelocity() - activeTargetVelocity) < VELO_TOL;
    }

    private void resetPID() {
        integralSum = 0;
        lastError   = 0;
        lastTime    = 0;
    }
}