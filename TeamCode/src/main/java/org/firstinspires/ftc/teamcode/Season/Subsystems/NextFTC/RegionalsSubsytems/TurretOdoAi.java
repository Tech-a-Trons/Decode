package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;

public class TurretOdoAi implements Subsystem {

    public static final TurretOdoAi INSTANCE = new TurretOdoAi();

    // =====================================================================
    // TWO-PHASE TRACKING STATE MACHINE
    //
    //  SEEKING
    //    • Odometry bearing drives turret to rough angle so tag enters FOV.
    //
    //  TRACKING — adaptive PI + heading-rate feedforward
    //    • FEEDFORWARD: every loop, the heading change since last loop is
    //      measured (degrees/sec). The turret is immediately moved that same
    //      amount so it stays on target PROACTIVELY during turns, before
    //      tx error even has time to build.
    //      Tune with HEADING_FF_GAIN: 1.0 = full compensation, 0.0 = none.
    //      Start at 1.0. If turret overshoots during turns, reduce slightly.
    //    • ADAPTIVE PI: handles residual error the feedforward doesn't cover.
    //      Gain scales with error size — aggressive when large, gentle when small.
    //    • currentServoPos is software-tracked — never use servo.getPosition().
    //
    // LIMELIGHT: mounted HORIZONTALLY → tx = getTargetXDegrees().
    //
    // TUNING ORDER:
    //   1. CAMERA_MOUNT_OFFSET_DEG=0, TX_SIGN_FLIP=1, HEADING_FF_GAIN=1.0
    //   2. Watch getLastTx() — should stay near 0 even while turning.
    //      If turret moves AWAY from tag → TX_SIGN_FLIP = -1.0
    //   3. If turret over-leads during turns → reduce HEADING_FF_GAIN (try 0.8)
    //   4. If residual offset remains at rest → increase TRACKING_KI slightly
    //   5. Adjust CAMERA_MOUNT_OFFSET_DEG until barrel centers on goal
    // =====================================================================

    private enum TurretState { SEEKING, TRACKING }
    private TurretState state = TurretState.SEEKING;

    public static int LOCK_CONFIRM_FRAMES = 1;
    public static int LOCK_LOST_FRAMES    = 50;

    private int consecutiveDetections = 0;
    private int consecutiveMisses     = 0;

    // ------------------ Hardware ------------------
    private Servo       turretServo1;
    private Servo       turretServo2;
    private Limelight3A limelight;

    // ------------------ Robot Pose (from odometry) ------------------
    private double x           = 0;
    private double y           = 0;
    private double heading     = 0;
    private double lastHeading = 0; // for heading rate calculation
    private double headingRate = 0; // degrees/sec, used for feedforward

    public double ManualAngleAdjust = 0;

    // ------------------ Target coordinates (odometry fallback) ------------------
    public static double xt = 130;
    public static double yt = 130;
    double AngleOffset = -30;

    // ------------------ Turret state ------------------
    private double targetAngleDeg   = 0;
    private double turretAngleDeg   = 0;
    private double distanceToTarget = 0;

    // Software-tracked servo position — ground truth.
    // NEVER replace with servo.getPosition() (returns last commanded, not physical).
    private double currentServoPos = 0.25;

    public static double SERVO_MIN = 0.0;
    public static double SERVO_MAX = 1.0;
    public boolean hardwareInitialized = false;

    public boolean manualMode = false;

    // =====================================================================
    // SEEKING PID CONSTANTS
    // =====================================================================
    public static double kP_seek      = 40;
    public static double kI           = 0.00;
    public static double kD           = 0.00;
    public static double MAX_VELOCITY = 1100;
    public static double TOLERANCE    = 0.1;
    public double AngleAdjust = 0;

    private double  lastError                 = 0;
    private double  integral                  = 0;
    private double  lastUpdateTime            = 0;
    private boolean firstRun                  = true;
    private double  commandedAngle            = 0;
    private boolean commandedAngleInitialized = false;

    private ElapsedTime loopTimer             = new ElapsedTime();
    private static final double MIN_LOOP_TIME = 0.010;
    private int skippedLoops = 0;

    // =====================================================================
    // LIMELIGHT / TRACKING CONFIG
    // =====================================================================
    public static int TARGET_TAG_ID = -1;

    // 340° over 0.0–1.0 servo range → servo units per degree
    private static final double SERVO_DEG_RATIO = 1.0 / 340.0;



    // ── HEADING RATE FEEDFORWARD ──────────────────────────────────────────
    // Separate gains per turn direction so each side tunes independently.
    // HEADING_FF_GAIN_RIGHT: robot turning right (positive headingRate) — WORKING, leave alone.
    // HEADING_FF_GAIN_LEFT:  robot turning left  (negative headingRate) — reduce if turret
    //                        overshoots right during left turns (try 0.6, 0.5, etc).
    public static double HEADING_FF_GAIN_RIGHT = 2.5; // right turns — do not change
    public static double HEADING_FF_GAIN_LEFT  = 20; // left turns  — tune this down
    // Handles residual error after feedforward.
    // Gain scales with error: large error → GAIN_MAX, small error → GAIN_MIN.
    public static double TRACKING_GAIN_MAX  = 5.0;
    public static double TRACKING_GAIN_MIN  = 0.5;
    public static double GAIN_THRESHOLD_DEG = 2.5;

    // I term — eliminates persistent residual offset that P alone leaves.
    public static double TRACKING_KI           = 0.0001;
    private double       trackingIntegral      = 0;
    private static final double TRACKING_I_MAX = 30.0;

    // Physical camera mount offset (camera left of barrel).
    public static double CAMERA_MOUNT_OFFSET_DEG = 7.0;

    // Fine crosshair bias trim.
    public static double TX_OFFSET = 0.0;

    // +1.0 or -1.0. Flip if turret moves AWAY from tag when locked.
    public static double TX_SIGN_FLIP = 1.0;

    // Dead-band — stops micro-corrections that cause jitter.
    public static double TX_TOLERANCE_DEG = 0.2;

    // Diagnostics
    private double lastTx                 = 0;
    private double lastTrackingServoDelta = 0;
    private double lastAdaptiveGain       = 0;
    private double lastFeedforward        = 0;

    private ElapsedTime llOrientationTimer              = new ElapsedTime();
    private static final double LL_ORIENTATION_INTERVAL = 0.1;

    private double lastPoseCorrectionMag    = 0;
    private int    limelightCorrectionCount = 0;
    public static double MAX_POSE_CORRECTION_INCHES = 12.0;

    private Pose cachedPose = null;

    private TurretOdoAi() {}

    // ------------------ Alliance Setup ------------------
    public void setAlliance(String alliance) {
        if (alliance.equals("blue")) {
            AngleOffset   = -30 + 90;
            TARGET_TAG_ID = 20;
            if (limelight != null) limelight.pipelineSwitch(5);
        } else if (alliance.equals("red")) {
            AngleOffset   = -36;
            xt            = 130;
            yt            = 130;
            TARGET_TAG_ID = 24;
            if (limelight != null) limelight.pipelineSwitch(4);
        }
    }

    // ------------------ Initialization ------------------
    public void init(HardwareMap hardwareMap) {
        try {
            turretServo1 = hardwareMap.get(Servo.class, "turretServo1");
            turretServo2 = hardwareMap.get(Servo.class, "turretServo2");
            limelight    = hardwareMap.get(Limelight3A.class, "Limelight");

            turretServo1.setPosition(0.25);
            turretServo2.setPosition(0.25);
            currentServoPos = 0.25;

            commandedAngle            = normalizeDegrees(servoToAngle(0.25));
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

    // ------------------ Main Loop ------------------
    @Override
    public void periodic() {
        if (!hardwareInitialized) return;

        double currentTime         = loopTimer.seconds();
        double timeSinceLastUpdate = currentTime - lastUpdateTime;
        if (timeSinceLastUpdate < MIN_LOOP_TIME) { skippedLoops++; return; }
        if (PedroComponent.follower() == null) return;

        try {
            // ================================================================
            // STEP 1 — Odometry pose + heading rate
            // ================================================================
            cachedPose = PedroComponent.follower().getPose();
            if (cachedPose == null) return;

            x = cachedPose.getX() - 72;
            y = cachedPose.getY() - 72;

            double newHeading = Math.toDegrees(cachedPose.getHeading());
            if (newHeading < 0) newHeading += 360;

            // Heading rate in degrees/sec — handles wrap-around
            double rawDelta = newHeading - lastHeading;
            if (rawDelta >  180) rawDelta -= 360;
            if (rawDelta < -180) rawDelta += 360;
            double dt_heading = timeSinceLastUpdate > 0 ? timeSinceLastUpdate : MIN_LOOP_TIME;
            headingRate = rawDelta / dt_heading; // degrees/sec

            lastHeading = newHeading;
            heading     = newHeading;

            // ================================================================
            // STEP 2 — Feed heading to Limelight (rate-limited)
            // ================================================================
            if (limelight != null && llOrientationTimer.seconds() > LL_ORIENTATION_INTERVAL) {
                limelight.updateRobotOrientation(heading);
                llOrientationTimer.reset();
            }

            // ================================================================
            // STEP 3 — Poll Limelight for the target AprilTag
            // ================================================================
            LLResultTypes.FiducialResult targetFiducial = null;
            if (limelight != null) {
                LLResult result = limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                    if (fiducials != null) {
                        for (LLResultTypes.FiducialResult f : fiducials) {
                            if (TARGET_TAG_ID < 0 || f.getFiducialId() == TARGET_TAG_ID) {
                                targetFiducial = f;
                                break;
                            }
                        }
                    }
                    if (targetFiducial != null) tryPoseCorrection(result);
                }
            }

            // ================================================================
            // STEP 4 — State machine transitions
            // ================================================================
            if (targetFiducial != null) {
                consecutiveDetections++;
                consecutiveMisses = 0;
                if (state == TurretState.SEEKING && consecutiveDetections >= LOCK_CONFIRM_FRAMES) {
                    state            = TurretState.TRACKING;
                    integral         = 0;
                    trackingIntegral = 0;
                    lastError        = 0;
                }
            } else {
                consecutiveMisses++;
                consecutiveDetections = 0;
                if (state == TurretState.TRACKING && consecutiveMisses >= LOCK_LOST_FRAMES) {
                    state            = TurretState.SEEKING;
                    integral         = 0;
                    trackingIntegral = 0;
                    lastError        = 0;
                }
            }

            // ================================================================
            // STEP 5 — TRACKING: feedforward + adaptive PI
            //
            // FEEDFORWARD (applied every loop, tag visible or not):
            //   Robot turned X degrees this loop → turret must turn -X degrees
            //   to maintain the same field-relative angle.
            //   This happens BEFORE we read tx, so the correction is proactive.
            //
            // ADAPTIVE PI (applied when tag visible):
            //   Corrects whatever residual error the feedforward didn't cover.
            //   Gain scales with error magnitude for fast chase + smooth settle.
            // ================================================================
            if (state == TurretState.TRACKING) {
                turretAngleDeg = servoToAngle(currentServoPos);

                // ── FEEDFORWARD: compensate for robot rotation this loop ──────
                // headingRate (deg/sec) * dt (sec) = degrees robot turned this loop
                // Turret must counter-rotate by that amount * FF gain.
                // The sign: if robot turns CW (positive heading rate in our convention),
                // the tag appears to move CCW in camera frame, so turret must turn CW.
                // TX_SIGN_FLIP handles if your servo direction is reversed.
                double headingDelta   = headingRate * timeSinceLastUpdate;
                // Pick gain based on turn direction — right is tuned, left has separate gain
                double ffGain        = (headingDelta >= 0) ? HEADING_FF_GAIN_RIGHT : HEADING_FF_GAIN_LEFT;
                double ffServoDelta   = TX_SIGN_FLIP * headingDelta * SERVO_DEG_RATIO * ffGain;
                lastFeedforward       = ffServoDelta;
                currentServoPos       = clamp(currentServoPos - ffServoDelta, SERVO_MIN, SERVO_MAX);

                // ── ADAPTIVE PI: correct residual tx error ────────────────────
                if (targetFiducial != null) {
                    double txDeg       = targetFiducial.getTargetXDegrees();
                    lastTx             = txDeg;
                    double offsetError = TX_SIGN_FLIP * (txDeg + TX_OFFSET - CAMERA_MOUNT_OFFSET_DEG);

                    if (Math.abs(offsetError) > TX_TOLERANCE_DEG) {
                        double absError     = Math.abs(offsetError);
                        double t            = Math.min(absError / GAIN_THRESHOLD_DEG, 1.0);
                        double adaptiveGain = TRACKING_GAIN_MIN + t * (TRACKING_GAIN_MAX - TRACKING_GAIN_MIN);
                        lastAdaptiveGain    = adaptiveGain;

                        trackingIntegral = clamp(trackingIntegral + offsetError, -TRACKING_I_MAX, TRACKING_I_MAX);

                        double piOutput   = (offsetError * adaptiveGain) + (TRACKING_KI * trackingIntegral);
                        double servoDelta = piOutput * SERVO_DEG_RATIO;

                        lastTrackingServoDelta = servoDelta;
                        currentServoPos        = clamp(currentServoPos - servoDelta, SERVO_MIN, SERVO_MAX);
                    } else {
                        trackingIntegral = 0;
                    }
                }
                // Tag absent: feedforward still ran above, PI skipped — hold residual

                turretServo1.setPosition(currentServoPos);
                turretServo2.setPosition(currentServoPos);
                commandedAngle = normalizeDegrees(servoToAngle(currentServoPos));

                lastUpdateTime = currentTime;
                return;
            }

            // ================================================================
            // STEP 6 — SEEKING: odometry bearing PID
            // ================================================================
            turretAngleDeg = servoToAngle(currentServoPos);

            if (!commandedAngleInitialized) {
                commandedAngle            = normalizeDegrees(turretAngleDeg);
                commandedAngleInitialized = true;
            }

            double dx = xt - x;
            double dy = yt - y;
            distanceToTarget = Math.sqrt(dx * dx + dy * dy);

            double fieldAngleDeg = Math.toDegrees(Math.atan2(dy, dx));
            if (fieldAngleDeg < 0) fieldAngleDeg += 360;

            targetAngleDeg = normalizeDegrees(
                    fieldAngleDeg - heading + 180 + AngleOffset + AngleAdjust + ManualAngleAdjust);

            double error = targetAngleDeg - commandedAngle;
            if (error >  180) error -= 360;
            if (error < -180) error += 360;

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
            // Never crash the opmode loop
        }
    }

    // ------------------ Pose Correction (opportunistic) ------------------
    private void tryPoseCorrection(LLResult result) {
        try {
            Pose3D botPose3D = result.getBotpose();
            if (botPose3D == null) return;

            double llX = botPose3D.getPosition().y / 0.0254 + 70.625;
            double llY = -botPose3D.getPosition().x / 0.0254 + 70.625;

            double dx = llX - (x + 72);
            double dy = llY - (y + 72);
            lastPoseCorrectionMag = Math.sqrt(dx * dx + dy * dy);

            if (lastPoseCorrectionMag > MAX_POSE_CORRECTION_INCHES) return;

            double llYawDeg = botPose3D.getOrientation().getYaw(AngleUnit.DEGREES) + 90.0;
            if (llYawDeg > 360) llYawDeg -= 360;
            if (llYawDeg < 0)   llYawDeg += 360;

            PedroComponent.follower().setPose(new Pose(llX, llY, Math.toRadians(llYawDeg)));
            limelightCorrectionCount++;
        } catch (Exception e) {
            // Silently ignore — pose correction is opportunistic
        }
    }

    // ------------------ Relocalization ------------------
    public boolean relocalize() {
        if (!hardwareInitialized || limelight == null || PedroComponent.follower() == null) return false;
        try {
            limelight.updateRobotOrientation(heading);
            LLResult result = limelight.getLatestResult();
            if (result == null || !result.isValid()) return false;
            if (result.getFiducialResults() == null || result.getFiducialResults().isEmpty()) return false;

            Pose3D botPose3D = result.getBotpose();
            if (botPose3D == null) return false;

            double llX = botPose3D.getPosition().y / 0.0254 + 70.625;
            double llY = -botPose3D.getPosition().x / 0.0254 + 70.625;

            double dx = llX - PedroComponent.follower().getPose().getX();
            double dy = llY - PedroComponent.follower().getPose().getY();
            if (Math.sqrt(dx * dx + dy * dy) > MAX_POSE_CORRECTION_INCHES) return false;

            double currentHeading = PedroComponent.follower().getPose().getHeading();
            PedroComponent.follower().setPose(new Pose(llX, llY, currentHeading));

            integral  = 0;
            lastError = 0;
            return true;
        } catch (Exception e) {
            return false;
        }
    }

    // ------------------ Helpers ------------------
    public void turnRight() { ManualAngleAdjust += 2; }
    public void turnLeft()  { ManualAngleAdjust -= 2; }

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

    // ------------------ Getters ------------------
    public double  getX()                        { return x; }
    public double  getY()                        { return y; }
    public double  getHeading()                  { return heading; }
    public double  getTargetAngleDeg()           { return targetAngleDeg; }
    public double  getTurretAngleDeg()           { return turretAngleDeg; }
    public double  getDistanceToTarget()         { return distanceToTarget; }
    public double  getLastError()                { return lastError; }
    public double  getIntegral()                 { return integral; }
    public double  getKp()                       { return (state == TurretState.TRACKING) ? 0 : kP_seek; }
    public int     getSkippedLoops()             { return skippedLoops; }
    public double  getLoopTime()                 { return loopTimer.seconds() - lastUpdateTime; }
    public boolean isManualMode()                { return manualMode; }
    public double  getCurrentServoPos()          { return currentServoPos; }
    public double  getCommandedAngle()           { return commandedAngle; }
    public boolean isLimelightLocked()           { return state == TurretState.TRACKING; }
    public int     getLimelightCorrectionCount() { return limelightCorrectionCount; }
    public double  getLastPoseCorrectionMag()    { return lastPoseCorrectionMag; }
    public TurretState getState()                { return state; }
    public int     getConsecutiveDetections()    { return consecutiveDetections; }
    public int     getConsecutiveMisses()        { return consecutiveMisses; }

    // Diagnostics — log in telemetry to debug tracking
    public double  getLastTx()                   { return lastTx; }
    public double  getLastTrackingServoDelta()   { return lastTrackingServoDelta; }
    public double  getLastAdaptiveGain()         { return lastAdaptiveGain; }
    public double  getTrackingIntegral()         { return trackingIntegral; }
    public double  getHeadingRate()              { return headingRate; }
    public double  getLastFeedforward()          { return lastFeedforward; }
}