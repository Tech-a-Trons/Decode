package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class ShooterPID {

    public static ShooterPID INSTANCE;

    // ── Tuning ────────────────────────────────────────────────────────────
    public static double kP = 0.007;    // Feedback - only trims remaining error
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kV = 0.00045;  // Feedforward velocity gain
    public static double kS = 0.02;     // Feedforward static friction offset
    public static double MAX_INTEGRAL = 0.2; // Anti-windup clamp
    public static double VELO_TOL = 75;

    // Always store positive target — motors run negative internally
    public static double activeTargetVelocity = 0;

    // ── Hardware ──────────────────────────────────────────────────────────
    private DcMotorEx outtakeLeft;
    private DcMotorEx outtakeRight;
    private VoltageSensor voltageSensor;

    // ── PID state ─────────────────────────────────────────────────────────
    private double integralSum = 0;
    private double lastError   = 0;
    private long   lastTime    = 0;

    // ── Private constructor — call init() from OpMode ─────────────────────
    private ShooterPID(HardwareMap hardwareMap) {
        outtakeLeft  = hardwareMap.get(DcMotorEx.class, "outtakeleft");
        outtakeRight = hardwareMap.get(DcMotorEx.class, "outtakeright");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

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

        // Integral with anti-windup clamp
        integralSum += error * dt;
        integralSum = Math.max(-MAX_INTEGRAL, Math.min(MAX_INTEGRAL, integralSum));

        double derivative = (error - lastError) / dt;
        lastError = error;

        // Feedforward carries the bulk of the load; PID only trims error
        double ff = kV * target + kS * Math.signum(target);
        double fb = kP * error + kI * integralSum + kD * derivative;

        // Voltage compensation: scale output so behavior is consistent as battery drains
        double voltage = voltageSensor.getVoltage();
        double output = (ff + fb) * (12.0 / voltage);
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