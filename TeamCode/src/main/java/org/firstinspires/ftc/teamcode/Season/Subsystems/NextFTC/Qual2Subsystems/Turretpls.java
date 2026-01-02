package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.CRServoEx;

@Config
public class Turretpls implements Subsystem {

    public static final Turret INSTANCE = new Turret();

    public static double kP = 0.6;
    public static double kI = 0.0;
    public static double kD = 0.04;

    public static double deadbandDeg = 0.6;
    public static double minPower = 0.08;
    public static double maxPower = 1.0;
    public static double maxAcceleration = 6.0;

    public static double positiveRangeMax = 185.0;
    public static double negativeRangeMax = -185.0;
    public static double wrapMarginDegrees = 8.0;

    private final CRServoEx turretServo;
    private final AnalogInput encoder;

    private double targetAngle = 0.0;
    private double currentAngle = 0.0;
    private double lastRawAngle = 0.0;

    private double lastError = 0.0;
    private double integralSum = 0.0;
    private double lastPower = 0.0;

    private double posWrapThreshold;
    private double negWrapThreshold;

    private boolean manual = false;
    private double manualPower = 0.0;

    public Turretpls() {
        turretServo = new CRServoEx("turret");
        encoder = hardwareMap.get(AnalogInput.class, "turret_encoder");
        posWrapThreshold = Math.toRadians(positiveRangeMax - wrapMarginDegrees);
        negWrapThreshold = Math.toRadians(negativeRangeMax + wrapMarginDegrees);
        lastRawAngle = getRawAngle();
    }

    public void setTargetDegrees(double degrees) {
        targetAngle = Math.toRadians(degrees);
        manual = false;
    }

    public void turn(double power) {
        manual = true;
        manualPower = power / 100.0;
    }

    private double getRawAngle() {
        return (encoder.getVoltage() / 3.3) * Math.PI * 2.0;
    }

    private double normalize(double angle) {
        while (angle > Math.PI) angle -= Math.PI * 2.0;
        while (angle < -Math.PI) angle += Math.PI * 2.0;
        return angle;
    }

    private void updateAngle() {
        double raw = getRawAngle();
        double diff = normalize(raw - lastRawAngle);
        currentAngle += diff;
        lastRawAngle = raw;
    }

    private void handleWrapping() {
        if (currentAngle > posWrapThreshold && targetAngle < 0) {
            targetAngle += Math.PI * 2.0;
        } else if (currentAngle < negWrapThreshold && targetAngle > 0) {
            targetAngle -= Math.PI * 2.0;
        }
    }

    @Override
    public void periodic() {
        updateAngle();

        if (manual) {
            lastPower = manualPower;
            turretServo.setPower(manualPower);
            return;
        }

        handleWrapping();

        double error = normalize(targetAngle - currentAngle);
        double errorDeg = Math.toDegrees(error);

        if (Math.signum(error) != Math.signum(lastError)) integralSum = 0.0;

        if (Math.abs(errorDeg) < deadbandDeg) {
            turretServo.setPower(0);
            lastPower = 0;
            integralSum = 0;
            lastError = error;
            return;
        }

        integralSum += error;
        double derivative = error - lastError;

        double power = kP * error + kI * integralSum + kD * derivative;
        power = Math.max(-maxPower, Math.min(maxPower, power));

        if (Math.abs(power) < minPower) power = Math.signum(power) * minPower;

        double delta = power - lastPower;
        double maxDelta = maxAcceleration / 50.0;
        if (Math.abs(delta) > maxDelta) power = lastPower + Math.signum(delta) * maxDelta;

        turretServo.setPower(power);
        lastPower = power;
        lastError = error;
    }
}
