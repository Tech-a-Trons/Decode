//package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems;
//
//import com.pedropathing.geometry.Pose;
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.LLResultTypes;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
//
//import java.util.List;
//
//import dev.nextftc.core.subsystems.Subsystem;
//import dev.nextftc.extensions.pedro.PedroComponent;
//
//public class TurretOdoAi implements Subsystem {
//
//    public static final TurretOdoAi INSTANCE = new TurretOdoAi();
//
//    // =====================================================================
//    // TWO-PHASE TRACKING STATE MACHINE
//    //
//    //  SEEKING
//    //    • Odometry bearing drives turret to rough angle so tag enters FOV.
//    //
//    //  TRACKING — adaptive PI + heading-rate feedforward
//    //    • FEEDFORWARD: every loop, the heading change since last loop is
//    //      measured (degrees/sec). The turret is immediately moved that same
//    //      amount so it stays on target PROACTIVELY during turns, before
//    //      tx error even has time to build.
//    //      Tune with HEADING_FF_GAIN: 1.0 = full compensation, 0.0 = none.
//    //      Start at 1.0. If turret overshoots during turns, reduce slightly.
//    //    • ADAPTIVE PI: handles residual error the feedforward doesn't cover.
//    //      Gain scales with error size — aggressive when large, gentle when small.
//    //    • currentServoPos is software-tracked — never use servo.getPosition().
//    //
//    // LIMELIGHT: mounted HORIZONTALLY → tx = getTargetXDegrees().
//    //
//    // TUNING ORDER:
//    //   1. CAMERA_MOUNT_OFFSET_DEG=0, TX_SIGN_FLIP=1, HEADING_FF_GAIN=1.0
//    //   2. Watch getLastTx() — should stay near 0 even while turning.
//    //      If turret moves AWAY from tag → TX_SIGN_FLIP = -1.0
//    //   3. If turret over-leads during turns → reduce HEADING_FF_GAIN (try 0.8)
//    //   4. If residual offset remains at rest → increase TRACKING_KI slightly
//    //   5. Adjust CAMERA_MOUNT_OFFSET_DEG until barrel centers on goal
//    // =====================================================================
//
//    private enum TurretState { SEEKING, TRACKING }
//    private TurretState state = TurretState.SEEKING;
//
//    public static int LOCK_CONFIRM_FRAMES = 1;
//    public static int LOCK_LOST_FRAMES    = 50;
//
//    private int consecutiveDetections = 0;
//    private int consecutiveMisses     = 0;
//
//    // ------------------ Hardware ------------------
//    private Servo       turretServo1;
//    private Servo       turretServo2;
//    private Limelight3A limelight;
//
//    // ------------------ Robot Pose (from odometry) ------------------
//    private double x           = 0;
//    private double y           = 0;
//    private double heading     = 0;
//    private double lastHeading = 0; // for heading rate calculation
//    private double headingRate = 0; // degrees/sec, used for feedforward
//
//    public double ManualAngleAdjust = 0;
//
//    // ------------------ Target coordinates (odometry fallback) ------------------
//    public static double xt = 130;
//    public static double yt = 130;
//    double AngleOffset = -30;
//
//    // ------------------ Turret state ------------------
//    private double targetAngleDeg   = 0;
//    private double turretAngleDeg   = 0;
//    private double distanceToTarget = 0;
//
//    // Software-tracked servo position — ground truth.
//    // NEVER replace with servo.getPosition() (returns last commanded, not physical).
//    private double currentServoPos = 0.25;
//
//    public static double SERVO_MIN = 0.0;
//    public static double SERVO_MAX = 1.0;
//    public boolean hardwareInitialized = false;
//
//    public boolean manualMode = false;
//
//    // =====================================================================
//    // SEEKING PID CONSTANTS
//    // =====================================================================
//    public static double kP_seek      = 40;
//    public static double kI           = 0.00;
//    public static double kD           = 0.00;
//    public static double MAX_VELOCITY = 1100;
//    public static double TOLERANCE    = 0.1;
//    public double AngleAdjust = 0;
//
//    private double  lastError                 = 0;
//    private double  integral                  = 0;
//    private double  lastUpdateTime            = 0;
//    private boolean firstRun                  = true;
//    private double  commandedAngle            = 0;
//    private boolean commandedAngleInitialized = false;
//
//    private ElapsedTime loopTimer             = new ElapsedTime();
//    private static final double MIN_LOOP_TIME = 0.010;
//    private int skippedLoops = 0;
//
//    // =====================================================================
//    // LIMELIGHT / TRACKING CONFIG
//    // =====================================================================
//    public static int TARGET_TAG_ID = -1;
//
//    // 340° over 0.0–1.0 servo range → servo units per degree
//    private static final double SERVO_DEG_RATIO = 1.0 / 340.0;
//
//
//
//    // ── HEADING RATE FEEDFORWARD ──────────────────────────────────────────
//    // When the robot turns, the turret counter-rotates by the same amount
//    // to keep the tag centered proactively, before tx error builds up.
//    //
//    // HEADING_FF_SIGN: direction of compensation. +1.0 or -1.0.
//    //   Separate from TX_SIGN_FLIP — controls FF direction only.
//    //   If FF makes tracking worse in BOTH directions → flip this.
//    //   Tune this before touching the gains.
//    public static double HEADING_FF_SIGN = 1.0;
//
//    // HEADING_FF_GAIN_RIGHT: proportional gain for right turns — WORKING, do not change.
//    // HEADING_FF_GAIN_LEFT:  proportional gain for left turns — base scaling.
//    public static double HEADING_FF_GAIN_RIGHT = 2.5; // do not change
//    public static double HEADING_FF_GAIN_LEFT  = 5.0;
//
//    // ── LEFT-ONLY THRESHOLD BOOST ─────────────────────────────────────────
//    // Extra fixed servo delta applied ONLY during left turns when heading rate
//    // exceeds LEFT_BOOST_THRESHOLD_DEG_S. Structurally different from the
//    // proportional gain — it's a flat kick based purely on turn speed, not tx.
//    // Start LEFT_BOOST_AMOUNT at 0.001 and increase until left catches up.
//    // Raise LEFT_BOOST_THRESHOLD_DEG_S if it triggers on small unintended drift.
//    public static double LEFT_BOOST_THRESHOLD_DEG_S = 2.5; // deg/sec to trigger boost
//    public static double LEFT_BOOST_AMOUNT          = 0.01; // extra servo units per loop
//    // Handles residual error after feedforward.
//    // Gain scales with error: large error → GAIN_MAX, small error → GAIN_MIN.
//    public static double TRACKING_GAIN_MAX  = 10;
//    public static double TRACKING_GAIN_MIN  = 0.5;
//    public static double GAIN_THRESHOLD_DEG = 2.5;
//
//    // I term — eliminates persistent residual offset that P alone leaves.
//    public static double TRACKING_KI           = 0.0001;
//    private double       trackingIntegral      = 0;
//    private static final double TRACKING_I_MAX = 30.0;
//
//    // Physical camera mount offset (camera left of barrel).
//    public static double CAMERA_MOUNT_OFFSET_DEG = 7.0;
//
//    // Fine crosshair bias trim.
//    public static double TX_OFFSET = 0.0;
//
//    // +1.0 or -1.0. Flip if turret moves AWAY from tag when locked.
//    public static double TX_SIGN_FLIP = 1.0;
//
//    // Dead-band — stops micro-corrections that cause jitter.
//    public static double TX_TOLERANCE_DEG = 0.2;
//
//    // Diagnostics
//    private double lastTx                 = 0;
//    private double lastTrackingServoDelta = 0;
//    private double lastAdaptiveGain       = 0;
//    private double lastFeedforward        = 0;
//
//    private ElapsedTime llOrientationTimer              = new ElapsedTime();
//    private static final double LL_ORIENTATION_INTERVAL = 0.1;
//
//    private double lastPoseCorrectionMag    = 0;
//    private int    limelightCorrectionCount = 0;
//    public static double MAX_POSE_CORRECTION_INCHES = 12.0;
//
//    private Pose cachedPose = null;
//
//    private TurretOdoAi() {}
//
//    // ------------------ Alliance Setup ------------------
//    public void setAlliance(String alliance) {
//        if (alliance.equals("blue")) {
//            AngleOffset   = -36 + 90-45;
//            TARGET_TAG_ID = 20;
//            if (limelight != null) limelight.pipelineSwitch(5);
//        } else if (alliance.equals("red")) {
//            AngleOffset   = -36-45;
//            xt            = 130;
//            yt            = 130;
//            TARGET_TAG_ID = 24;
//            if (limelight != null) limelight.pipelineSwitch(4);
//        }
//    }
//
//    // ------------------ Initialization ------------------
//    public void init(HardwareMap hardwareMap) {
//        try {
//            turretServo1 = hardwareMap.get(Servo.class, "turretServo1");
//            turretServo2 = hardwareMap.get(Servo.class, "turretServo2");
//            limelight    = hardwareMap.get(Limelight3A.class, "Limelight");
//
//            turretServo1.setPosition(0.25);
//            turretServo2.setPosition(0.25);
//            currentServoPos = 0.25;
//
//            commandedAngle            = normalizeDegrees(servoToAngle(0.25));
//            commandedAngleInitialized = true;
//
//            loopTimer.reset();
//            llOrientationTimer.reset();
//            lastUpdateTime = loopTimer.seconds();
//            firstRun       = true;
//
//            state                 = TurretState.SEEKING;
//            consecutiveDetections = 0;
//            consecutiveMisses     = 0;
//            trackingIntegral      = 0;
//            headingRate           = 0;
//
//            hardwareInitialized = true;
//        } catch (Exception e) {
//            hardwareInitialized = false;
//        }
//    }
//
//    // ------------------ Main Loop ------------------
//    @Override
//    public void periodic() {
//        if (!hardwareInitialized) return;
//
//        double currentTime         = loopTimer.seconds();
//        double timeSinceLastUpdate = currentTime - lastUpdateTime;
//        if (timeSinceLastUpdate < MIN_LOOP_TIME) { skippedLoops++; return; }
//        if (PedroComponent.follower() == null) return;
//
//        try {
//            // ================================================================
//            // STEP 1 — Odometry pose + heading rate
//            // ================================================================
//            cachedPose = PedroComponent.follower().getPose();
//            if (cachedPose == null) return;
//
//            x = cachedPose.getX() - 72;
//            y = cachedPose.getY() - 72;
//
//            double newHeading = Math.toDegrees(cachedPose.getHeading());
//            if (newHeading < 0) newHeading += 360;
//
//            // Heading rate in degrees/sec — handles wrap-around
//            double rawDelta = newHeading - lastHeading;
//            if (rawDelta >  180) rawDelta -= 360;
//            if (rawDelta < -180) rawDelta += 360;
//            double dt_heading = timeSinceLastUpdate > 0 ? timeSinceLastUpdate : MIN_LOOP_TIME;
//            headingRate = rawDelta / dt_heading; // degrees/sec
//
//            lastHeading = newHeading;
//            heading     = newHeading;
//
//            // ================================================================
//            // STEP 2 — Feed heading to Limelight (rate-limited)
//            // ================================================================
//            if (limelight != null && llOrientationTimer.seconds() > LL_ORIENTATION_INTERVAL) {
//                limelight.updateRobotOrientation(heading);
//                llOrientationTimer.reset();
//            }
//
//            // ================================================================
//            // STEP 3 — Poll Limelight for the target AprilTag
//            // ================================================================
//            LLResultTypes.FiducialResult targetFiducial = null;
//            if (limelight != null) {
//                LLResult result = limelight.getLatestResult();
//                if (result != null && result.isValid()) {
//                    List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
//                    if (fiducials != null) {
//                        for (LLResultTypes.FiducialResult f : fiducials) {
//                            if (TARGET_TAG_ID < 0 || f.getFiducialId() == TARGET_TAG_ID) {
//                                targetFiducial = f;
//                                break;
//                            }
//                        }
//                    }
//                    if (targetFiducial != null) tryPoseCorrection(result);
//                }
//            }
//
//            // ================================================================
//            // STEP 4 — State machine transitions
//            // ================================================================
//            if (targetFiducial != null) {
//                consecutiveDetections++;
//                consecutiveMisses = 0;
//                if (state == TurretState.SEEKING && consecutiveDetections >= LOCK_CONFIRM_FRAMES) {
//                    state            = TurretState.TRACKING;
//                    integral         = 0;
//                    trackingIntegral = 0;
//                    lastError        = 0;
//                }
//            } else {
//                consecutiveMisses++;
//                consecutiveDetections = 0;
//                if (state == TurretState.TRACKING && consecutiveMisses >= LOCK_LOST_FRAMES) {
//                    state            = TurretState.SEEKING;
//                    integral         = 0;
//                    trackingIntegral = 0;
//                    lastError        = 0;
//                }
//            }
//
//            // ================================================================
//            // STEP 5 — TRACKING: feedforward + adaptive PI
//            //
//            // FEEDFORWARD (applied every loop, tag visible or not):
//            //   Robot turned X degrees this loop → turret must turn -X degrees
//            //   to maintain the same field-relative angle.
//            //   This happens BEFORE we read tx, so the correction is proactive.
//            //
//            // ADAPTIVE PI (applied when tag visible):
//            //   Corrects whatever residual error the feedforward didn't cover.
//            //   Gain scales with error magnitude for fast chase + smooth settle.
//            // ================================================================
//            if (state == TurretState.TRACKING) {
//                turretAngleDeg = servoToAngle(currentServoPos);
//
//                // ── FEEDFORWARD: compensate for robot rotation this loop ──────
//                // headingRate (deg/sec) * dt (sec) = degrees robot turned this loop
//                // Turret must counter-rotate by that amount * FF gain.
//                // The sign: if robot turns CW (positive heading rate in our convention),
//                // the tag appears to move CCW in camera frame, so turret must turn CW.
//                // TX_SIGN_FLIP handles if your servo direction is reversed.
//                double headingDelta = headingRate * timeSinceLastUpdate;
//                double ffGain       = (headingDelta >= 0) ? HEADING_FF_GAIN_RIGHT : HEADING_FF_GAIN_LEFT;
//                double ffServoDelta = HEADING_FF_SIGN * headingDelta * SERVO_DEG_RATIO * ffGain;
//
//                // Left-only threshold boost — flat extra kick during fast left turns
//                if (headingRate < -LEFT_BOOST_THRESHOLD_DEG_S) {
//                    ffServoDelta += HEADING_FF_SIGN * LEFT_BOOST_AMOUNT;
//                }
//                lastFeedforward       = ffServoDelta;
//                currentServoPos       = clamp(currentServoPos - ffServoDelta, SERVO_MIN, SERVO_MAX);
//
//                // ── ADAPTIVE PI: correct residual tx error ────────────────────
//                if (targetFiducial != null) {
//                    double txDeg       = targetFiducial.getTargetXDegrees();
//                    lastTx             = txDeg;
//                    double offsetError = TX_SIGN_FLIP * (txDeg + TX_OFFSET - CAMERA_MOUNT_OFFSET_DEG);
//
//                    if (Math.abs(offsetError) > TX_TOLERANCE_DEG) {
//                        double absError     = Math.abs(offsetError);
//                        double t            = Math.min(absError / GAIN_THRESHOLD_DEG, 1.0);
//                        double adaptiveGain = TRACKING_GAIN_MIN + t * (TRACKING_GAIN_MAX - TRACKING_GAIN_MIN);
//                        lastAdaptiveGain    = adaptiveGain;
//
//                        trackingIntegral = clamp(trackingIntegral + offsetError, -TRACKING_I_MAX, TRACKING_I_MAX);
//
//                        double piOutput   = (offsetError * adaptiveGain) + (TRACKING_KI * trackingIntegral);
//                        double servoDelta = piOutput * SERVO_DEG_RATIO;
//
//                        lastTrackingServoDelta = servoDelta;
//                        currentServoPos        = clamp(currentServoPos - servoDelta, SERVO_MIN, SERVO_MAX);
//                    } else {
//                        trackingIntegral = 0;
//                    }
//                }
//                // Tag absent: feedforward still ran above, PI skipped — hold residual
//
//                turretServo1.setPosition(currentServoPos);
//                turretServo2.setPosition(currentServoPos);
//                commandedAngle = normalizeDegrees(servoToAngle(currentServoPos));
//
//                lastUpdateTime = currentTime;
//                return;
//            }
//
//            // ================================================================
//            // STEP 6 — SEEKING: odometry bearing PID
//            // ================================================================
//            turretAngleDeg = servoToAngle(currentServoPos);
//
//            if (!commandedAngleInitialized) {
//                commandedAngle            = normalizeDegrees(turretAngleDeg);
//                commandedAngleInitialized = true;
//            }
//
//            double dx = xt - x;
//            double dy = yt - y;
//            distanceToTarget = Math.sqrt(dx * dx + dy * dy);
//
//            double fieldAngleDeg = Math.toDegrees(Math.atan2(dy, dx));
//            if (fieldAngleDeg < 0) fieldAngleDeg += 360;
//
//            targetAngleDeg = normalizeDegrees(
//                    fieldAngleDeg - heading + 180 + AngleOffset + AngleAdjust + ManualAngleAdjust);
//
//            double error = targetAngleDeg - commandedAngle;
//            if (error >  180) error -= 360;
//            if (error < -180) error += 360;
//
//            if (Math.abs(error) < 0.5) {
//                lastError      = error;
//                lastUpdateTime = currentTime;
//                return;
//            }
//
//            double dt = firstRun ? MIN_LOOP_TIME : timeSinceLastUpdate;
//            if (dt <= 0 || dt > 0.2) dt = MIN_LOOP_TIME;
//            firstRun = false;
//
//            double P_output  = kP_seek * error;
//            integral        += error * dt;
//            integral         = clamp(integral, -100, 100);
//            double I_output  = kI * integral;
//            double D_output  = kD * ((error - lastError) / dt);
//            double pidOutput = clamp(P_output + I_output + D_output, -MAX_VELOCITY, MAX_VELOCITY);
//
//            commandedAngle  = normalizeDegrees(commandedAngle + pidOutput * dt);
//            currentServoPos = clamp(angleToServo(commandedAngle), SERVO_MIN, SERVO_MAX);
//
//            turretServo1.setPosition(currentServoPos);
//            turretServo2.setPosition(currentServoPos);
//
//            lastError      = error;
//            lastUpdateTime = currentTime;
//
//        } catch (Exception e) {
//            // Never crash the opmode loop
//        }
//    }
//
//    // ------------------ Pose Correction (opportunistic) ------------------
//    private void tryPoseCorrection(LLResult result) {
//        try {
//            Pose3D botPose3D = result.getBotpose();
//            if (botPose3D == null) return;
//
//            double llX = botPose3D.getPosition().y / 0.0254 + 70.625;
//            double llY = -botPose3D.getPosition().x / 0.0254 + 70.625;
//
//            double dx = llX - (x + 72);
//            double dy = llY - (y + 72);
//            lastPoseCorrectionMag = Math.sqrt(dx * dx + dy * dy);
//
//            if (lastPoseCorrectionMag > MAX_POSE_CORRECTION_INCHES) return;
//
//            double llYawDeg = botPose3D.getOrientation().getYaw(AngleUnit.DEGREES) + 90.0;
//            if (llYawDeg > 360) llYawDeg -= 360;
//            if (llYawDeg < 0)   llYawDeg += 360;
//
//            PedroComponent.follower().setPose(new Pose(llX, llY, Math.toRadians(llYawDeg)));
//            limelightCorrectionCount++;
//        } catch (Exception e) {
//            // Silently ignore — pose correction is opportunistic
//        }
//    }
//
//    // ------------------ Relocalization ------------------
//    public boolean relocalize() {
//        if (!hardwareInitialized || limelight == null || PedroComponent.follower() == null) return false;
//        try {
//            limelight.updateRobotOrientation(heading);
//            LLResult result = limelight.getLatestResult();
//            if (result == null || !result.isValid()) return false;
//            if (result.getFiducialResults() == null || result.getFiducialResults().isEmpty()) return false;
//
//            Pose3D botPose3D = result.getBotpose();
//            if (botPose3D == null) return false;
//
//            double llX = botPose3D.getPosition().y / 0.0254 + 70.625;
//            double llY = -botPose3D.getPosition().x / 0.0254 + 70.625;
//
//            double dx = llX - PedroComponent.follower().getPose().getX();
//            double dy = llY - PedroComponent.follower().getPose().getY();
//            if (Math.sqrt(dx * dx + dy * dy) > MAX_POSE_CORRECTION_INCHES) return false;
//
//            double currentHeading = PedroComponent.follower().getPose().getHeading();
//            PedroComponent.follower().setPose(new Pose(llX, llY, currentHeading));
//
//            integral  = 0;
//            lastError = 0;
//            return true;
//        } catch (Exception e) {
//            return false;
//        }
//    }
//
//    // ------------------ Helpers ------------------
//    public void turnRight() { ManualAngleAdjust += 20; }
//    public void turnLeft()  { ManualAngleAdjust -= 20; }
//
//    private double angleToServo(double angleDeg) {
//        angleDeg = normalizeDegrees(angleDeg);
//        return clamp(1 - ((angleDeg + 180) / 340.0), SERVO_MIN, SERVO_MAX);
//    }
//
//    private double servoToAngle(double servoPos) {
//        return normalizeDegrees(340.0 * (1.0 - servoPos) - 180.0);
//    }
//
//    public double normalizeDegrees(double angle) {
//        while (angle >  180) angle -= 360;
//        while (angle < -180) angle += 360;
//        return angle;
//    }
//
//    private double clamp(double val, double min, double max) {
//        return Math.max(min, Math.min(max, val));
//    }
//
//    // ------------------ Getters ------------------
//    public double  getX()                        { return x; }
//    public double  getY()                        { return y; }
//    public double  getHeading()                  { return heading; }
//    public double  getTargetAngleDeg()           { return targetAngleDeg; }
//    public double  getTurretAngleDeg()           { return turretAngleDeg; }
//    public double  getDistanceToTarget()         { return distanceToTarget; }
//    public double  getLastError()                { return lastError; }
//    public double  getIntegral()                 { return integral; }
//    public double  getKp()                       { return (state == TurretState.TRACKING) ? 0 : kP_seek; }
//    public int     getSkippedLoops()             { return skippedLoops; }
//    public double  getLoopTime()                 { return loopTimer.seconds() - lastUpdateTime; }
//    public boolean isManualMode()                { return manualMode; }
//    public double  getCurrentServoPos()          { return currentServoPos; }
//    public double  getCommandedAngle()           { return commandedAngle; }
//    public boolean isLimelightLocked()           { return state == TurretState.TRACKING; }
//    public int     getLimelightCorrectionCount() { return limelightCorrectionCount; }
//    public double  getLastPoseCorrectionMag()    { return lastPoseCorrectionMag; }
//    public TurretState getState()                { return state; }
//    public int     getConsecutiveDetections()    { return consecutiveDetections; }
//    public int     getConsecutiveMisses()        { return consecutiveMisses; }
//
//    // Diagnostics — log in telemetry to debug tracking
//    public double  getLastTx()                   { return lastTx; }
//    public double  getLastTrackingServoDelta()   { return lastTrackingServoDelta; }
//    public double  getLastAdaptiveGain()         { return lastAdaptiveGain; }
//    public double  getTrackingIntegral()         { return trackingIntegral; }
//    public double  getHeadingRate()              { return headingRate; }
//    public double  getLastFeedforward()          { return lastFeedforward; }
//}
package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;

/**
 * TurretOdoAi — two-phase turret tracking:
 *
 *  SEEKING  — odometry bearing PID sweeps the turret until the tag enters FOV.
 *  TRACKING — 2D tx (getTargetXDegrees) + heading-rate feedforward keeps it centered.
 *             Odometry is completely frozen while TRACKING — Limelight owns the servo.
 *
 * No MegaTag2, no botpose, no pose correction. Pure 2D fiducial tx.
 *
 * TUNING ORDER:
 *  1. TX_SIGN_FLIP = 1.0, CAMERA_MOUNT_OFFSET_DEG = 0, HEADING_FF_GAIN_RIGHT/LEFT = 0
 *  2. Verify tag appears in FOV and state switches to TRACKING.
 *  3. Watch lastTx — should converge toward 0. If turret moves AWAY → TX_SIGN_FLIP = -1.0
 *  4. Add HEADING_FF_GAIN_RIGHT = 2.5, then tune HEADING_FF_GAIN_LEFT separately.
 *  5. Dial CAMERA_MOUNT_OFFSET_DEG until barrel centers on goal at rest.
 */
public class TurretOdoAi implements Subsystem {

    public static final TurretOdoAi INSTANCE = new TurretOdoAi();
    private TurretOdoAi() {}

    // =====================================================================
    // STATE MACHINE
    // =====================================================================
    public enum TurretState { SEEKING, TRACKING }
    private TurretState state = TurretState.SEEKING;

    // Frames before switching SEEKING → TRACKING (raise if false positives)
    public static int LOCK_CONFIRM_FRAMES = 2;
    // Frames of no detection before switching TRACKING → SEEKING
    public static int LOCK_LOST_FRAMES    = 50;

    private int consecutiveDetections = 0;
    private int consecutiveMisses     = 0;

    // =====================================================================
    // HARDWARE
    // =====================================================================
    private Servo       turretServo1;
    private Servo       turretServo2;
    private Limelight3A limelight;
    public  boolean     hardwareInitialized = false;

    // =====================================================================
    // SERVO CONFIG
    // =====================================================================
    // 340° of turret travel over 0.0–1.0 servo range
    private static final double SERVO_DEG_RATIO = 1.0 / 340.0;
    public static double SERVO_MIN = 0.0;
    public static double SERVO_MAX = 1.0;

    // Software-tracked position — NEVER read back with servo.getPosition()
    private double currentServoPos = 0.5;

    // =====================================================================
    // ODOMETRY / SEEKING CONFIG
    // =====================================================================
    public static double xt = 130; // field target X (inches)
    public static double yt = 130; // field target Y (inches)

    // Angle offset tuned per alliance — adjusts bearing so turret faces goal
    private double AngleOffset = -36 - 45;

    public double AngleAdjust       = 0;
    public double ManualAngleAdjust = 0;

    // Seeking PID
    public static double kP_seek      = 40.0;
    public static double kI           = 0.0;
    public static double kD           = 0.0;
    public static double MAX_VELOCITY = 1100.0;

    private double  commandedAngle            = 0;
    private boolean commandedAngleInitialized = false;
    private double  integral                  = 0;
    private double  lastError                 = 0;
    private boolean firstRun                  = true;

    // =====================================================================
    // TRACKING CONFIG — 2D TX
    // =====================================================================
    // Tag to track. -1 = any tag.
    public static int TARGET_TAG_ID = 24;

    // +1.0 or -1.0. Flip if turret moves AWAY from tag.
    public static double TX_SIGN_FLIP = 1.0;

    // Physical camera-to-barrel offset in degrees.
    public static double CAMERA_MOUNT_OFFSET_DEG = 7.0;

    // Fine crosshair trim.
    public static double TX_OFFSET = 0.0;

    // Dead-band — suppresses jitter when nearly centered.
    public static double TX_TOLERANCE_DEG = 0.2;

    // ── Adaptive PI ──────────────────────────────────────────────────────
    // Gain interpolates between MIN and MAX based on error magnitude.
    public static double TRACKING_GAIN_MAX  = 10.0;
    public static double TRACKING_GAIN_MIN  = 0.5;
    public static double GAIN_THRESHOLD_DEG = 2.5;

    public static double TRACKING_KI           = 0.0001;
    private double       trackingIntegral      = 0;
    private static final double TRACKING_I_MAX = 30.0;

    // ── Heading-rate feedforward ──────────────────────────────────────────
    // Compensates for robot rotation so turret stays on target proactively.
    // Tune RIGHT first (it's working), then tune LEFT independently.
    public static double HEADING_FF_SIGN       = 1.0;
    public static double HEADING_FF_GAIN_RIGHT = 2.5; // robot turning right (headingDelta >= 0)
    public static double HEADING_FF_GAIN_LEFT  = 5.0; // robot turning left  (headingDelta < 0)

    // Extra flat boost for fast left turns — increase if left still lags.
    public static double LEFT_BOOST_THRESHOLD_DEG_S = 2.5;
    public static double LEFT_BOOST_AMOUNT          = 0.01;

    // =====================================================================
    // LOOP TIMING
    // =====================================================================
    private ElapsedTime         loopTimer      = new ElapsedTime();
    private static final double MIN_LOOP_TIME  = 0.010; // 100 Hz cap
    private double              lastUpdateTime = 0;
    private int                 skippedLoops   = 0;

    private ElapsedTime         llOrientationTimer             = new ElapsedTime();
    private static final double LL_ORIENTATION_INTERVAL        = 0.1;

    // =====================================================================
    // ROBOT POSE (from odometry)
    // =====================================================================
    private double x           = 0;
    private double y           = 0;
    private double heading     = 0;
    private double lastHeading = 0;
    private double headingRate = 0; // degrees/sec

    // Derived
    private double targetAngleDeg   = 0;
    private double turretAngleDeg   = 0;
    private double distanceToTarget = 0;

    // =====================================================================
    // DIAGNOSTICS
    // =====================================================================
    private double lastTx                 = 0;
    private double lastTrackingServoDelta = 0;
    private double lastAdaptiveGain       = 0;
    private double lastFeedforward        = 0;

    private Pose cachedPose = null;

    // =====================================================================
    // ALLIANCE SETUP
    // =====================================================================
    public void setAlliance(String alliance) {
        if (alliance.equalsIgnoreCase("blue")) {
            AngleOffset   = -36 + 90 - 45;
            TARGET_TAG_ID = 20;
            if (limelight != null) limelight.pipelineSwitch(0);
        } else if (alliance.equalsIgnoreCase("red")) {
            AngleOffset   = -45 - 45;
            xt            = 130;
            yt            = 130;
            TARGET_TAG_ID = 24;
            if (limelight != null) limelight.pipelineSwitch(1);
        }
    }

    // =====================================================================
    // INITIALIZATION
    // =====================================================================
    public void init(HardwareMap hardwareMap) {
        try {
            turretServo1 = hardwareMap.get(Servo.class, "turretServo1");
            turretServo2 = hardwareMap.get(Servo.class, "turretServo2");
            limelight    = hardwareMap.get(Limelight3A.class, "Limelight");

            currentServoPos = 0.5;
            turretServo1.setPosition(currentServoPos);
            turretServo2.setPosition(currentServoPos);

            commandedAngle            = normalizeDegrees(servoToAngle(currentServoPos));
            commandedAngleInitialized = true;

            loopTimer.reset();
            llOrientationTimer.reset();
            lastUpdateTime = loopTimer.seconds();
            firstRun       = true;

            state                 = TurretState.SEEKING;
            consecutiveDetections = 0;
            consecutiveMisses     = 0;
            trackingIntegral      = 0;
            headingRate           = 0;

            hardwareInitialized = true;
        } catch (Exception e) {
            hardwareInitialized = false;
        }
    }

    // =====================================================================
    // MAIN LOOP
    // =====================================================================
    @Override
    public void periodic() {
        if (!hardwareInitialized) return;

        double currentTime         = loopTimer.seconds();
        double timeSinceLastUpdate = currentTime - lastUpdateTime;
        if (timeSinceLastUpdate < MIN_LOOP_TIME) { skippedLoops++; return; }
        if (PedroComponent.follower() == null) return;

        try {
            // ============================================================
            // STEP 1 — Odometry pose + heading rate
            // Always read pose even in TRACKING so heading rate stays fresh
            // for the feedforward. We just don't use x/y/bearing in TRACKING.
            // ============================================================
            cachedPose = PedroComponent.follower().getPose();
            if (cachedPose == null) return;

            x = cachedPose.getX() - 72;
            y = cachedPose.getY() - 72;

            double newHeading = Math.toDegrees(cachedPose.getHeading());
            if (newHeading < 0) newHeading += 360;

            double rawDelta = newHeading - lastHeading;
            if (rawDelta >  180) rawDelta -= 360;
            if (rawDelta < -180) rawDelta += 360;
            double dtHeading = timeSinceLastUpdate > 0 ? timeSinceLastUpdate : MIN_LOOP_TIME;
            headingRate  = rawDelta / dtHeading; // deg/sec
            lastHeading  = newHeading;
            heading      = newHeading;

            // ============================================================
            // STEP 2 — Feed heading to Limelight (rate-limited)
            // ============================================================
            if (limelight != null && llOrientationTimer.seconds() > LL_ORIENTATION_INTERVAL) {
                limelight.updateRobotOrientation(heading);
                llOrientationTimer.reset();
            }

            // ============================================================
            // STEP 3 — Poll Limelight for 2D fiducial tx
            // ============================================================
            LLResultTypes.FiducialResult targetFiducial = getTargetFiducial();

            // ============================================================
            // STEP 4 — State machine transitions
            // ============================================================
            if (targetFiducial != null) {
                consecutiveDetections++;
                consecutiveMisses = 0;
                if (state == TurretState.SEEKING && consecutiveDetections >= LOCK_CONFIRM_FRAMES) {
                    state            = TurretState.TRACKING;
                    trackingIntegral = 0;
                    integral         = 0;
                    lastError        = 0;
                }
            } else {
                consecutiveMisses++;
                consecutiveDetections = 0;
                if (state == TurretState.TRACKING && consecutiveMisses >= LOCK_LOST_FRAMES) {
                    state            = TurretState.SEEKING;
                    trackingIntegral = 0;
                    integral         = 0;
                    lastError        = 0;
                    // Re-sync commandedAngle to current servo so seeking
                    // resumes from wherever tracking left off, no jump.
                    commandedAngle = normalizeDegrees(servoToAngle(currentServoPos));
                }
            }

            // ============================================================
            // STEP 5 — TRACKING: heading-rate feedforward + adaptive PI
            //
            // Odometry bearing is COMPLETELY IGNORED here.
            // The servo is owned entirely by Limelight tx.
            // ============================================================
            if (state == TurretState.TRACKING) {
                turretAngleDeg = servoToAngle(currentServoPos);

                // ── Feedforward: counter-rotate for robot heading change ──
                double headingDelta = headingRate * timeSinceLastUpdate;
                double ffGain       = (headingDelta >= 0) ? HEADING_FF_GAIN_RIGHT : HEADING_FF_GAIN_LEFT;
                double ffServoDelta = HEADING_FF_SIGN * headingDelta * SERVO_DEG_RATIO * ffGain;

                // Extra flat boost for fast left turns
                if (headingRate < -LEFT_BOOST_THRESHOLD_DEG_S) {
                    ffServoDelta += HEADING_FF_SIGN * LEFT_BOOST_AMOUNT;
                }

                lastFeedforward = ffServoDelta;
                currentServoPos = clamp(currentServoPos - ffServoDelta, SERVO_MIN, SERVO_MAX);

                // ── Adaptive PI: correct residual 2D tx error ────────────
                if (targetFiducial != null) {
                    // getTargetXDegrees() = raw 2D tx — no MegaTag2
                    double txDeg      = targetFiducial.getTargetXDegrees();
                    lastTx            = txDeg;

                    double offsetError = TX_SIGN_FLIP * (txDeg + TX_OFFSET - CAMERA_MOUNT_OFFSET_DEG);

                    if (Math.abs(offsetError) > TX_TOLERANCE_DEG) {
                        double absError     = Math.abs(offsetError);
                        double t            = Math.min(absError / GAIN_THRESHOLD_DEG, 1.0);
                        double adaptiveGain = TRACKING_GAIN_MIN + t * (TRACKING_GAIN_MAX - TRACKING_GAIN_MIN);
                        lastAdaptiveGain    = adaptiveGain;

                        trackingIntegral = clamp(
                                trackingIntegral + offsetError,
                                -TRACKING_I_MAX, TRACKING_I_MAX);

                        double piOutput   = (offsetError * adaptiveGain) + (TRACKING_KI * trackingIntegral);
                        double servoDelta = piOutput * SERVO_DEG_RATIO;

                        lastTrackingServoDelta = servoDelta;
                        currentServoPos        = clamp(currentServoPos - servoDelta, SERVO_MIN, SERVO_MAX);
                    } else {
                        trackingIntegral = 0;
                    }
                }
                // Tag temporarily absent: feedforward still ran, PI holds last correction

                turretServo1.setPosition(currentServoPos);
                turretServo2.setPosition(currentServoPos);

                // Keep commandedAngle in sync so seeking resumes smoothly if lock lost
                commandedAngle = normalizeDegrees(servoToAngle(currentServoPos));

                lastUpdateTime = currentTime;
                return; // ← odometry code below is completely skipped
            }

            // ============================================================
            // STEP 6 — SEEKING: odometry bearing PID
            // Only runs when Limelight has NO vision of the target.
            // ============================================================
            turretAngleDeg = servoToAngle(currentServoPos);

            if (!commandedAngleInitialized) {
                commandedAngle            = normalizeDegrees(turretAngleDeg);
                commandedAngleInitialized = true;
            }

            double dx = xt - x;
            double dy = yt - y;
            distanceToTarget = Math.hypot(dx, dy);

            double fieldAngleDeg = Math.toDegrees(Math.atan2(dy, dx));
            if (fieldAngleDeg < 0) fieldAngleDeg += 360;

            targetAngleDeg = normalizeDegrees(
                    fieldAngleDeg - heading + 180 + AngleOffset + AngleAdjust + ManualAngleAdjust);

            double error = targetAngleDeg - commandedAngle;
            if (error >  180) error -= 360;
            if (error < -180) error += 360;

            // Already close enough — hold position
            if (Math.abs(error) < 0.5) {
                lastError      = error;
                lastUpdateTime = currentTime;
                return;
            }

            double dt = firstRun ? MIN_LOOP_TIME : timeSinceLastUpdate;
            if (dt <= 0 || dt > 0.2) dt = MIN_LOOP_TIME;
            firstRun = false;

            double P_output  = kP_seek * error;
            integral        += error * dt;
            integral         = clamp(integral, -100, 100);
            double I_output  = kI * integral;
            double D_output  = kD * ((error - lastError) / dt);
            double pidOutput = clamp(P_output + I_output + D_output, -MAX_VELOCITY, MAX_VELOCITY);

            commandedAngle  = normalizeDegrees(commandedAngle + pidOutput * dt);
            currentServoPos = clamp(angleToServo(commandedAngle), SERVO_MIN, SERVO_MAX);

            turretServo1.setPosition(currentServoPos);
            turretServo2.setPosition(currentServoPos);

            lastError      = error;
            lastUpdateTime = currentTime;

        } catch (Exception e) {
            // Never crash the OpMode loop
        }
    }

    // =====================================================================
    // HELPERS
    // =====================================================================
    private LLResultTypes.FiducialResult getTargetFiducial() {
        if (limelight == null) return null;
        try {
            LLResult result = limelight.getLatestResult();
            if (result == null || !result.isValid()) return null;

            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials == null || fiducials.isEmpty()) return null;

            for (LLResultTypes.FiducialResult f : fiducials) {
                if (TARGET_TAG_ID < 0 || f.getFiducialId() == TARGET_TAG_ID) return f;
            }
        } catch (Exception e) { /* ignore */ }
        return null;
    }

    public void turnRight() { ManualAngleAdjust += 20; }
    public void turnLeft()  { ManualAngleAdjust -= 20; }

    private double angleToServo(double angleDeg) {
        angleDeg = normalizeDegrees(angleDeg);
        return clamp(1 - ((angleDeg + 180) / 340.0), SERVO_MIN, SERVO_MAX);
    }

    private double servoToAngle(double servoPos) {
        return normalizeDegrees(340.0 * (1.0 - servoPos) - 180.0);
    }

    public double normalizeDegrees(double angle) {
        while (angle >  180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    // =====================================================================
    // GETTERS
    // =====================================================================
    public double     getX()                        { return x; }
    public double     getY()                        { return y; }
    public double     getHeading()                  { return heading; }
    public double     getTargetAngleDeg()           { return targetAngleDeg; }
    public double     getTurretAngleDeg()           { return turretAngleDeg; }
    public double     getDistanceToTarget()         { return distanceToTarget; }
    public double     getLastError()                { return lastError; }
    public double     getIntegral()                 { return integral; }
    public int        getSkippedLoops()             { return skippedLoops; }
    public double     getCurrentServoPos()          { return currentServoPos; }
    public double     getCommandedAngle()           { return commandedAngle; }
    public boolean    isLimelightLocked()           { return state == TurretState.TRACKING; }
    public TurretState getState()                   { return state; }
    public int        getConsecutiveDetections()    { return consecutiveDetections; }
    public int        getConsecutiveMisses()        { return consecutiveMisses; }
    public double     getLastTx()                   { return lastTx; }
    public double     getLastTrackingServoDelta()   { return lastTrackingServoDelta; }
    public double     getLastAdaptiveGain()         { return lastAdaptiveGain; }
    public double     getTrackingIntegral()         { return trackingIntegral; }
    public double     getHeadingRate()              { return headingRate; }
    public double     getLastFeedforward()          { return lastFeedforward; }
}