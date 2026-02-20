package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class OuttakePID {

    public static OuttakePID INSTANCE;

    // PID constants (matching TurretPID)
    public static double kP = 0.004;
    public static double kI = 0.0;
    public static double kD = 0.0;

    public static double VELO_TOL = 75;
    public static double activeTargetVelocity = 0;

    private DcMotorEx outtakeLeft;
    private DcMotorEx outtakeRight;

    // PID state
    private double integralSum = 0;
    private double lastError = 0;
    private long lastTime = 0;

    private OuttakePID(HardwareMap hardwareMap) {
        outtakeLeft  = hardwareMap.get(DcMotorEx.class, "outtakeleft");
        outtakeRight = hardwareMap.get(DcMotorEx.class, "outtakeright");

        // outtakeright is reversed to match MotorEx("outtakeright").reversed()
        outtakeLeft.setDirection(DcMotor.Direction.FORWARD);
        outtakeRight.setDirection(DcMotor.Direction.REVERSE);

        outtakeLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        outtakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public static void init(HardwareMap hardwareMap) {
        INSTANCE = new OuttakePID(hardwareMap);
    }

    // Call this every loop iteration from your OpMode
    public void update() {
        double currentVelocity = getActualVelocity();
        double error = activeTargetVelocity - currentVelocity;

        long now = System.nanoTime();
        double dt = (now - lastTime) / 1e9; // seconds
        if (lastTime == 0 || dt <= 0) dt = 0.02; // fallback on first loop
        lastTime = now;

        integralSum += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        double output = kP * error + kI * integralSum + kD * derivative;

        // Clamp output to [-1, 1]
        output = Math.max(-1.0, Math.min(1.0, output));

        outtakeLeft.setPower(output);
        outtakeRight.setPower(output);
    }

    public void setTargetVelocity(double velocity) {
        activeTargetVelocity = velocity;
        resetPID();
    }

    public void stop() {
        activeTargetVelocity = 0;
        resetPID();
        outtakeLeft.setPower(0);
        outtakeRight.setPower(0);
    }

    public boolean atTargetVelocity() {
        return Math.abs(getActualVelocity() - activeTargetVelocity) < VELO_TOL;
    }

    public double getActualVelocity() {
        return (outtakeLeft.getVelocity() + outtakeRight.getVelocity()) / 2.0;
    }

    public double getTargetVelocity() {
        return activeTargetVelocity;
    }

    private void resetPID() {
        integralSum = 0;
        lastError = 0;
        lastTime = 0;
    }
}