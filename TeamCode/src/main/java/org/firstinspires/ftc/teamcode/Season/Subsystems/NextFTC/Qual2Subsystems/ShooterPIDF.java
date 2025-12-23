package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterPIDF {

    private DcMotor leftMotor;
    private DcMotor rightMotor;

    private double kP = 0.05;
    private double kI = 0.0;
    private double kD = 0.0;
    private double kV = 0.002;
    private double kA = 0.0;
    private double kS = 0.01;

    private double targetVelocity = 0;
    private double integral = 0;
    private double lastError = 0;

    private int lastLeftPos = 0;
    private int lastRightPos = 0;
    private long lastTime = 0;

    public ShooterPIDF(HardwareMap hwMap, String leftName, String rightName) {
        leftMotor = hwMap.get(DcMotor.class, leftName);
        rightMotor = hwMap.get(DcMotor.class, rightName);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lastLeftPos = leftMotor.getCurrentPosition();
        lastRightPos = rightMotor.getCurrentPosition();
        lastTime = System.currentTimeMillis();
    }

    public void setTargetVelocity(double target) {
        targetVelocity = target;
    }

    public void update() {
        long now = System.currentTimeMillis();
        double dt = (now - lastTime) / 1000.0;
        if (dt == 0) return;
        lastTime = now;

        int leftPos = leftMotor.getCurrentPosition();
        int rightPos = rightMotor.getCurrentPosition();

        double leftVel = (leftPos - lastLeftPos) / dt;
        double rightVel = (rightPos - lastRightPos) / dt;
        lastLeftPos = leftPos;
        lastRightPos = rightPos;

        double currentVel = (leftVel + rightVel) / 2.0;

        double error = targetVelocity - currentVel;
        integral += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        double power = kP * error
                + kI * integral
                + kD * derivative
                + kV * targetVelocity
                + kA * ((currentVel - lastError)/dt)
                + kS * Math.signum(targetVelocity);

        power = Math.max(-1.0, Math.min(1.0, power));

        leftMotor.setPower(power);
        rightMotor.setPower(power); // reverse the right motor
    }
}