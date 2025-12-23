//package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.pedropathing.geometry.Pose;
//
//import dev.nextftc.core.subsystems.Subsystem;
//import dev.nextftc.hardware.impl.ServoEx;
//
//@Config
//public class OdoTurretSubsystem implements Subsystem {
//
//    public static final OdoTurretSubsystem INSTANCE = new OdoTurretSubsystem();
//
//    public static double center = 0.5;
//    public static double range = Math.PI;
//    public static double servoMin = 0.0, servoMax = 1.0;
//
//    private final ServoEx turret;
//    private boolean manual = false;
//    private double manualPower = 0;
//    private double targetYaw = 0; // radians
//    private boolean enabled = true;
//
//    // Optional: store a target pose if you want continuous tracking
//    private Pose targetPose = null;
//    private Pose robotPose = null;
//
//    public OdoTurretSubsystem() {
//        turret = new ServoEx("turret");
//        turret.setPosition(center);
//    }
//
//    public void setYaw(double radians) {
//        targetYaw = normalizeAngle(radians);
//    }
//
//    public void addYaw(double radians) {
//        setYaw(getYaw() + radians);
//    }
//
//    public double getYaw() {
//        double pos = turret.getPosition();
//        return normalizeAngle((pos - center) * range);
//    }
//
//    public void manual(double power) {
//        manual = true;
//        manualPower = power;
//    }
//
//    public void automatic() {
//        manual = false;
//    }
//
//    public void on() {
//        enabled = true;
//    }
//
//    public void off() {
//        enabled = false;
//    }
//
//    // Set the robot and target poses for auto-tracking
//    public void setTargetPose(Pose targetPose, Pose robotPose) {
//        this.targetPose = targetPose;
//        this.robotPose = robotPose;
//        if (targetPose != null && robotPose != null) {
//            face(targetPose, robotPose);
//        }
//    }
//
//    public void face(Pose targetPose, Pose robotPose) {
//        double angleToTarget = Math.atan2(targetPose.getY() - robotPose.getY(),
//                targetPose.getX() - robotPose.getX());
//        double relative = normalizeAngle(angleToTarget - robotPose.getHeading());
//        setYaw(relative);
//    }
//
//    public void setPosition(double radians) {
//        setYaw(radians);
//    }
//
//    public void addPosition(double radians) {
//        addYaw(radians);
//    }
//
//    @Override
//    public void periodic() {
//        if (!enabled) return;
//
//        // If a target pose is set, update targetYaw
//        if (targetPose != null && robotPose != null) {
//            face(targetPose, robotPose);
//        }
//
//        double servoPos;
//        if (manual) {
//            servoPos = center + manualPower * 0.5;
//        } else {
//            double error = normalizeAngle(targetYaw - getYaw());
//            double power = error / range;
//            power = clamp(power, -1, 1);
//            servoPos = center + power * 0.5;
//        }
//
//        servoPos = clamp(servoPos, servoMin, servoMax);
//        turret.setPosition(servoPos);
//    }
//
//    public static double normalizeAngle(double angleRadians) {
//        double angle = angleRadians % (2 * Math.PI);
//        if (angle <= -Math.PI) angle += 2 * Math.PI;
//        if (angle > Math.PI) angle -= 2 * Math.PI;
//        return angle;
//    }
//
//    private double clamp(double val, double min, double max) {
//        return Math.max(min, Math.min(max, val));
//    }
//}
package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;

import dev.nextftc.extensions.pedro.PedroComponent;

@Config
public class OdoTurretSubsystem implements Subsystem {

    public static final OdoTurretSubsystem INSTANCE = new OdoTurretSubsystem();

    public static double center = 0.5;
    public static double range = Math.PI;
    public static double servoMin = 0.0, servoMax = 1.0;

    private final ServoEx turret;
    private boolean manual = false;
    private double manualPower = 0;
    private double targetYaw = 0;
    private boolean autoTrack = false;

    private Pose currentGoal = new Pose(129, 129, 0); // default goal

    private OdoTurretSubsystem() {
        turret = new ServoEx("turret");
        turret.setPosition(center);
    }

    public void manual(double power) {
        manual = true;
        manualPower = power;
    }

    public void automatic() {
        manual = false;
    }

    public void setGoal(Pose goalPose) {
        currentGoal = goalPose;
    }

    public void toggleAutoTrack() {
        autoTrack = !autoTrack;
    }

    public void setYaw(double radians) {
        targetYaw = normalizeAngle(radians);
    }

    public double getYaw() {
        return normalizeAngle((turret.getPosition() - center) * range);
    }

    @Override
    public void periodic() {
        // Track only if enabled
        if (autoTrack) {
            Pose robotPose = PedroComponent.follower().getPose();; // provided automatically by PedroComponent
            double angleToTarget = Math.atan2(
                    currentGoal.getY() - robotPose.getY(),
                    currentGoal.getX() - robotPose.getX()
            );
            double relative = normalizeAngle(angleToTarget - robotPose.getHeading());
            targetYaw = relative;
        }

        double servoPos;
        if (manual) {
            servoPos = center + manualPower * 0.5;
        } else {
            double error = normalizeAngle(targetYaw - getYaw());
            double power = error / range;
            power = clamp(power, -1, 1);
            servoPos = center + power * 0.5;
        }

        turret.setPosition(clamp(servoPos, servoMin, servoMax));
    }

    public static double normalizeAngle(double angle) {
        double a = angle % (2 * Math.PI);
        if (a <= -Math.PI) a += 2 * Math.PI;
        if (a > Math.PI) a -= 2 * Math.PI;
        return a;
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}