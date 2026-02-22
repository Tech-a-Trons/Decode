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
    //    • Odometry calculates the bearing from the robot to the target and
    //      drives the turret to that rough angle FIRST.
    //    • This is intentional: the odometry sweep is what puts the AprilTag
    //      inside the Limelight's FOV so it can take over.
    //    • Transitions to TRACKING once the Limelight has seen the correct
    //      AprilTag for LOCK_CONFIRM_FRAMES consecutive frames.
    //
    //  TRACKING
    //    • The Limelight's tx (horizontal offset in degrees) has FINAL say.
    //    • targetAngleDeg = commandedAngle + tx_corrected
    //    • Tag temporarily lost?
    //        → HOLD the last commanded angle (no odometry jerk).
    //        → After LOCK_LOST_FRAMES consecutive misses → SEEKING.
    //
    // CAMERA MOUNT OFFSET:
    //   The Limelight is mounted on the LEFT side of the turret.
    //   When tx = 0, the camera crosshair is centred on the tag, but the
    //   turret centreline is offset by CAMERA_MOUNT_OFFSET_DEG to the right.
    //   We subtract the mount offset from every tx reading so the turret
    //   aligns its centreline — not just the camera crosshair — with the tag.
    //   Tune CAMERA_MOUNT_OFFSET_DEG on the field (start at 0, increase until
    //   the turret is visually centred on the goal when isLimelightLocked()).
    // =====================================================================

    private enum TurretState { SEEKING, TRACKING }
    private TurretState state = TurretState.SEEKING;

    // Consecutive valid frames required to confirm lock (SEEKING → TRACKING).
    public static int LOCK_CONFIRM_FRAMES = 3;

    // Consecutive missed frames before falling back (TRACKING → SEEKING).
    // Intentionally high to survive brief occlusions.
    public static int LOCK_LOST_FRAMES = 10;

    private int consecutiveDetections = 0;
    private int consecutiveMisses     = 0;

    // ------------------ Hardware ------------------
    private Servo       turretServo1;
    private Servo       turretServo2;
    private Limelight3A limelight;

    // ------------------ Robot Pose (from odometry) ------------------
    private double x       = 0;
    private double y       = 0;
    private double heading = 0;

    public double ManualAngleAdjust = 0;

    // ------------------ Target coordinates (odometry fallback) ------------------
    public static double xt = 130;
    public static double yt = 130;
    double AngleOffset = -30;

    // ------------------ Turret state ------------------
    private double targetAngleDeg   = 0;
    private double turretAngleDeg   = 0;
    private double distanceToTarget = 0;
    private double currentServoPos  = 0;

    public static double SERVO_MIN = 0.0;
    public static double SERVO_MAX = 1.0;
    public boolean hardwareInitialized = false;

    public boolean manualMode     = false;
    private double manualPosition = 0.25;

    // =====================================================================
    // PID CONSTANTS
    //   SEEKING  → kP_seek  (aggressive, sweeps to rough odometry angle)
    //   TRACKING → kP_track (gentle,     fine-centres on the tag)
    // =====================================================================
    public static double kP_seek  = 1.0;
    public static double kP_track = 0.5;
    public static double kI           = 0.001;
    public static double kD           = 0.04;
    public static double MAX_VELOCITY = 1100;
    public static double TOLERANCE    = 0.1;
    public double AngleAdjust = 0;

    // PID state
    private double  lastError                 = 0;
    private double  integral                  = 0;
    private double  lastUpdateTime            = 0;
    private boolean firstRun                  = true;
    private double  commandedAngle            = 0;
    private boolean commandedAngleInitialized = false;

    // Rate limiting
    private ElapsedTime loopTimer             = new ElapsedTime();
    private static final double MIN_LOOP_TIME = 0.010;
    private int skippedLoops = 0;

    // =====================================================================
    // LIMELIGHT CONFIG
    // =====================================================================
    public static int TARGET_TAG_ID = -1;

    // Physical camera mount offset (LEFT-side mount).
    // Positive = camera is rotated left of the turret centreline.
    // Tune on field: with isLimelightLocked() true, increment this until
    // the turret centres on the goal, not just the camera crosshair.
    public static double CAMERA_MOUNT_OFFSET_DEG = 5.0;

    // Fine crosshair bias trim (small residual after mount offset is set).
    public static double TX_OFFSET = 0.0;

    // Set to -1.0 if the turret moves the wrong direction when tracking.
    public static double TX_SIGN_FLIP = 1.0;

    // Dead-band in TRACKING mode — don't move for tiny residual tx.
    public static double TX_TOLERANCE_DEG = 0.5;

    // Limelight heading update rate limiting
    private ElapsedTime llOrientationTimer              = new ElapsedTime();
    private static final double LL_ORIENTATION_INTERVAL = 0.1;

    // Diagnostics
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
            xt = 130;
            yt = 130;
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
            manualPosition = 0.25;

            commandedAngle            = normalizeDegrees(servoToAngle(0.25));
            commandedAngleInitialized = true;

            loopTimer.reset();
            llOrientationTimer.reset();
            lastUpdateTime = loopTimer.seconds();
            firstRun       = true;

            state                 = TurretState.SEEKING;
            consecutiveDetections = 0;
            consecutiveMisses     = 0;

            hardwareInitialized = true;
        } catch (Exception e) {
            hardwareInitialized = false;
        }
    }

    // ------------------ Manual Control ------------------
    public void incrementPosition(double delta) {
        if (!hardwareInitialized) return;
        manualPosition = clamp(manualPosition + delta, SERVO_MIN, SERVO_MAX);
        if (Math.abs(turretServo1.getPosition() - manualPosition) > 0.002) {
            turretServo1.setPosition(manualPosition);
            turretServo2.setPosition(manualPosition);
            commandedAngle = normalizeDegrees(servoToAngle(manualPosition));
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
            // STEP 1 — Odometry pose update
            //   Always runs — heading is needed even in TRACKING mode,
            //   and the bearing calculation runs whenever we are SEEKING.
            // ================================================================
            cachedPose = PedroComponent.follower().getPose();
            if (cachedPose == null) return;

            x       = cachedPose.getX() - 72;
            y       = cachedPose.getY() - 72;
            heading = Math.toDegrees(cachedPose.getHeading());
            if (heading < 0) heading += 360;

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
                    // Opportunistically improve odometry whenever the tag is visible
                    if (targetFiducial != null) tryPoseCorrection(result);
                }
            }

            // ================================================================
            // STEP 4 — State machine transitions
            // ================================================================
            if (targetFiducial != null) {
                consecutiveDetections++;
                consecutiveMisses = 0;

                // SEEKING → TRACKING
                // Odometry has already swept the turret close enough for the
                // camera to see the tag. Now lock on with the Limelight.
                if (state == TurretState.SEEKING && consecutiveDetections >= LOCK_CONFIRM_FRAMES) {
                    state = TurretState.TRACKING;
                    integral  = 0;   // clear seeking wind-up
                    lastError = 0;
                }
            } else {
                consecutiveMisses++;
                consecutiveDetections = 0;

                // TRACKING → SEEKING (only after sustained loss)
                if (state == TurretState.TRACKING && consecutiveMisses >= LOCK_LOST_FRAMES) {
                    state = TurretState.SEEKING;
                    integral  = 0;
                    lastError = 0;
                }
                // While consecutiveMisses < LOCK_LOST_FRAMES we remain in
                // TRACKING and will HOLD position (Case C below).
            }

            // ================================================================
            // STEP 5 — Compute targetAngleDeg
            //
            //  Case A  SEEKING               → odometry bearing
            //  Case B  TRACKING + tag seen   → Limelight tx  (final say)
            //  Case C  TRACKING + tag absent → hold last position
            // ================================================================
            currentServoPos = turretServo1.getPosition();
            turretAngleDeg  = servoToAngle(currentServoPos);

            if (!commandedAngleInitialized) {
                commandedAngle            = normalizeDegrees(turretAngleDeg);
                commandedAngleInitialized = true;
            }

            if (state == TurretState.SEEKING) {
                // ── Case A: ODOMETRY ─────────────────────────────────────
                // Compute the field-relative bearing to (xt, yt), convert to
                // a turret-relative angle, and drive toward it.
                // The purpose is to put the tag inside the camera's FOV so
                // the Limelight can confirm lock and take over.
                double dx = xt - x;
                double dy = yt - y;
                distanceToTarget = Math.sqrt(dx * dx + dy * dy);

                double fieldAngleDeg = Math.toDegrees(Math.atan2(dy, dx));
                if (fieldAngleDeg < 0) fieldAngleDeg += 360;

                targetAngleDeg = normalizeDegrees(
                        fieldAngleDeg - heading + 180 + AngleOffset + AngleAdjust + ManualAngleAdjust);

            } else if (targetFiducial != null) {
                // ── Case B: LIMELIGHT — final say ────────────────────────
                // tx = degrees the tag is left/right of the camera crosshair.
                //
                // Camera mount correction:
                //   Camera is on the LEFT of the turret centreline.
                //   CAMERA_MOUNT_OFFSET_DEG is subtracted so the turret aligns
                //   its centreline with the tag, not just the camera crosshair.
                //   Example: if the camera is 5° left of centre and tx = 0,
                //   the tag appears centred in the camera but the turret is
                //   actually 5° right of the goal — subtracting 5° corrects this.
                //
                // We apply tx to commandedAngle (not the physical servo read-back)
                // to avoid lag-induced oscillation while the servo is catching up.
                double tx = (targetFiducial.getTargetXDegrees() + TX_OFFSET)
                        - CAMERA_MOUNT_OFFSET_DEG;
                targetAngleDeg = normalizeDegrees(
                        commandedAngle + TX_SIGN_FLIP * tx + AngleAdjust + ManualAngleAdjust);

            } else {
                // ── Case C: HOLD ─────────────────────────────────────────
                // Still in TRACKING but tag briefly absent.
                // Zero the error so the PID holds the current position.
                // The turret won't jerk to an odometry estimate on a dropout.
                targetAngleDeg = commandedAngle;
            }

            // ================================================================
            // STEP 6 — PID drive toward targetAngleDeg
            // ================================================================
            double error = targetAngleDeg - commandedAngle;
            if (error >  180) error -= 360;
            if (error < -180) error += 360;

            // Dead-band. Save lastError even on early return to prevent a
            // derivative spike on the next active frame.
            double tolerance = (state == TurretState.TRACKING) ? TX_TOLERANCE_DEG : 0.5;
            if (Math.abs(error) < tolerance) {
                lastError      = error;
                lastUpdateTime = currentTime;
                return;
            }

            double dt = firstRun ? MIN_LOOP_TIME : timeSinceLastUpdate;
            if (dt <= 0 || dt > 0.2) dt = MIN_LOOP_TIME;
            firstRun = false;

            double kP = (state == TurretState.TRACKING) ? kP_track : kP_seek;

            double P_output = kP * error;

            integral += error * dt;
            if (Math.abs(error) < TOLERANCE) integral = 0;
            integral = clamp(integral, -100, 100);
            double I_output = kI * integral;

            double derivative = (error - lastError) / dt;
            double D_output   = kD * derivative;

            double pidOutput = clamp(P_output + I_output + D_output, -MAX_VELOCITY, MAX_VELOCITY);

            commandedAngle += pidOutput * dt;
            commandedAngle  = normalizeDegrees(commandedAngle);
            double newServoPos = clamp(angleToServo(commandedAngle), SERVO_MIN, SERVO_MAX);

            if (Math.abs(newServoPos - currentServoPos) > 0.002) {
                turretServo1.setPosition(newServoPos);
                turretServo2.setPosition(newServoPos);
            }

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
    public double  getKp()                       { return (state == TurretState.TRACKING) ? kP_track : kP_seek; }
    public int     getSkippedLoops()             { return skippedLoops; }
    public double  getLoopTime()                 { return loopTimer.seconds() - lastUpdateTime; }
    public boolean isManualMode()                { return manualMode; }
    public double  getManualPosition()           { return manualPosition; }
    public double  getCommandedAngle()           { return commandedAngle; }
    public boolean isLimelightLocked()           { return state == TurretState.TRACKING; }
    public int     getLimelightCorrectionCount() { return limelightCorrectionCount; }
    public double  getLastPoseCorrectionMag()    { return lastPoseCorrectionMag; }
    public TurretState getState()                { return state; }
    public int     getConsecutiveDetections()    { return consecutiveDetections; }
    public int     getConsecutiveMisses()        { return consecutiveMisses; }
}