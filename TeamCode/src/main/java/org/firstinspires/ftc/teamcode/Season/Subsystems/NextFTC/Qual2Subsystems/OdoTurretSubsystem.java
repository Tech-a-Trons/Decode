//package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems;
//
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.math.Vector;
//
//import dev.nextftc.core.subsystems.Subsystem;
//import dev.nextftc.hardware.impl.ServoEx;
//import dev.nextftc.extensions.pedro.PedroComponent;
//
//public class OdoTurretSubsystem implements Subsystem {
//
//    public static final OdoTurretSubsystem INSTANCE = new OdoTurretSubsystem();
//
//    public static double range = Math.PI;
//    public static final double GEAR_RATIO = 80.0 / 200.0; // 0.4
//    public double debugTargetYaw = 0;
//    public double debugServoCmd = 0;
//    public double debugServoActual = 0;
//    public double debugRobotX = 0;
//    public double debugRobotY = 0;
//    public double debugRobotHeading = 0;
//
//    private ServoEx turret; // lazy init
//
//    private double targetYaw = 0;
//    public boolean autoTrack = false;
//
//    private Pose currentGoal = new Pose(129, 129, 0);
//
//    private OdoTurretSubsystem() { }
//
//    // Call this from onStartButtonPressed() in your OpMode
//    public void init() {
//        if (turret == null) {
//            turret = new ServoEx("turret");
//            turret.setPosition(0.5); // center hardware position
//        }
//    }
//
//    public void setGoal(Pose goalPose) {
//        currentGoal = goalPose;
//    }
//
//    public void toggleAutoTrack() {
//        autoTrack = !autoTrack;
//    }
//
//    @Override
//    public void periodic() {
//        if (turret == null) return;
//
//        PedroComponent.follower().update();
//
//        Pose robotPose = PedroComponent.follower().getPose();
//        if (!autoTrack || robotPose == null) return;
//
//        // OPTIONAL: velocity-corrected pose (Pedro-style)
//        Pose currPose = robotPose;
//        Vector vel = PedroComponent.follower().getVelocity();
//        if (vel != null) {
//            currPose = new Pose(
//                    currPose.getX() + vel.getXComponent(),
//                    currPose.getY() + vel.getYComponent(),
//                    currPose.getHeading()
//            );
//        }
//
//        // Absolute angle to goal
//        double angleToGoal = Math.atan2(
//                currentGoal.getY() - currPose.getY(),
//                currentGoal.getX() - currPose.getX()
//        );
//
//        // Relative turret angle
//        targetYaw = normalizeAngle(angleToGoal - currPose.getHeading());
//
//        // Apply gear ratio (turret rotates slower than robot frame)
//        double turretAngle = targetYaw * GEAR_RATIO;
//
//        // Convert radians → Axon [-1, 1]
//        double servoCmd = clamp(turretAngle / Math.PI, -1, 1);
//
//        // Axon MAX servo-mode → [0, 1]
//        double hwPos = (servoCmd + 1.0) * 0.5;
//        turret.setPosition(hwPos);
//
//        // Debug
//        debugTargetYaw = targetYaw;
//        debugServoCmd = servoCmd;
//        debugServoActual = turret.getPosition();
//        debugRobotX = currPose.getX();
//        debugRobotY = currPose.getY();
//        debugRobotHeading = currPose.getHeading();
//    }
//
//    private static double normalizeAngle(double angle) {
//        double a = angle % (2 * Math.PI);
//        if (a <= -Math.PI) a += 2 * Math.PI;
//        if (a > Math.PI) a -= 2 * Math.PI;
//        return a;
//    }
//
//    private static double clamp(double val, double min, double max) {
//        return Math.max(min, Math.min(max, val));
//    }
//}