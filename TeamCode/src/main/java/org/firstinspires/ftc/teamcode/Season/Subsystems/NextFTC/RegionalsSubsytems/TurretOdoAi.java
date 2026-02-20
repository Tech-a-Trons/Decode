package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;

public class TurretOdoAi implements Subsystem {

    public static final TurretOdoAi INSTANCE = new TurretOdoAi();

    // ------------------ Hardware ------------------
    private Servo turretServo1;
    private Servo turretServo2;
    Limelight3A limelight;

    // ------------------ Robot Pose ------------------
    private double x = 0;
    private double y = 0;
    private double heading = 0;

    public double AngleAdjust = 0;

    // ------------------ Target (Red Goal) ------------------
    public static double xt = 130;
    public static double yt = 130;

    double AngleOffset = -30;


    // ------------------ Turret ------------------
    private double targetAngleDeg = 0;
    private double turretAngleDeg = 0;
    private double distanceToTarget = 0;

    // Servo safety
    public static double SERVO_MIN = 0.0;
    public static double SERVO_MAX = 1.0;
    public boolean hardwareInitialized = false;

    // Manual mode
    public boolean manualMode = false;
    private double manualPosition = 0.25;

    // ========== PID CONSTANTS ==========
    public static double kP = 10.00;
    public static double kI = 0.001;
    public static double kD = 0.04;

    double currentServoPos = 0;

    // Motion limits
    public static double MAX_VELOCITY = 1100;
    public static double TOLERANCE = 0.1;

    // ========== PID STATE VARIABLES ==========
    private double lastError = 0;
    private double integral = 0;
    private double lastUpdateTime = 0;
    private boolean firstRun = true;

    // ========== RATE LIMITING ==========
    private ElapsedTime loopTimer = new ElapsedTime();
    private static final double MIN_LOOP_TIME = 0.010;
    private int skippedLoops = 0;

    // ========== LIMELIGHT CORRECTION ==========
    private ElapsedTime limelightCorrectionTimer = new ElapsedTime();
    public static double LIMELIGHT_CORRECTION_INTERVAL = 0.5; // seconds between corrections
    public static double MAX_POSE_CORRECTION_INCHES = 12.0;   // sanity check — reject wild jumps
    public static double MAX_ANGLE_CORRECTION_DEG  = 20.0;   // max AngleAdjust delta per correction
    public static boolean limelightCorrectionEnabled = true;

    // Diagnostics
    private double lastLimelightCorrectionTime = -1;
    private int    limelightCorrectionCount    = 0;
    private double lastPoseCorrectionMag       = 0;
    private double lastAngleCorrectionDelta    = 0;
    private boolean lastCorrectionSucceeded    = false;

    // ========== IMPROVED WRAPPING FIX ==========
    private double commandedAngle = 0;
    private boolean commandedAngleInitialized = false;

    // ========== PERFORMANCE OPTIMIZATIONS ==========
    private Pose cachedPose = null;
    private static final double DEG_TO_RAD = Math.PI / 180.0;
    private static final double RAD_TO_DEG = 180.0 / Math.PI;

    private TurretOdoAi() {
    }

    public void setAlliance(String alliance) {
        if (alliance.equals("blue")) {
            AngleOffset = -30 + 90;
        }
        if (alliance.equals("red")) {
            AngleOffset = -36;
            xt = 130;
            yt = 130;
        }
    }

    // ------------------ Initialization ------------------
    public void init(HardwareMap hardwareMap) {
        try {
            turretServo1 = hardwareMap.get(Servo.class, "turretServo1");
            turretServo2 = hardwareMap.get(Servo.class, "turretServo2");

            limelight = hardwareMap.get(Limelight3A.class, "Limelight");

            // Set safe initial position
            turretServo1.setPosition(0.25);
            turretServo2.setPosition(0.25);
            manualPosition = 0.25;

            // Initialize commanded angle tracking — normalize on init
            commandedAngle = normalizeDegrees(servoToAngle(0.25));
            commandedAngleInitialized = true;

            // Initialize timers
            loopTimer.reset();
            lastUpdateTime = loopTimer.seconds();
            firstRun = true;

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

            // Update commanded angle when manually moving — normalize to prevent drift
            commandedAngle = normalizeDegrees(servoToAngle(manualPosition));
        }
    }


    // ------------------ Loop ------------------
    @Override
    public void periodic() {
        // === EARLY EXIT: Manual mode lightweight update ===
        // === EARLY EXIT: Hardware check ===
        if (!hardwareInitialized) return;

        // === EARLY EXIT: Rate limiting ===
        double currentTime = loopTimer.seconds();
        double timeSinceLastUpdate = currentTime - lastUpdateTime;

        if (timeSinceLastUpdate < MIN_LOOP_TIME) {
            skippedLoops++;
            return;
        }

        // === OPTIMIZED: Single follower null check ===
        if (PedroComponent.follower() == null) return;

        try {
            // === OPTIMIZED: Get pose once and cache ===
            cachedPose = PedroComponent.follower().getPose();
            if (cachedPose == null) return;

            // === OPTIMIZED: Direct field access instead of getters ===
            x = cachedPose.getX() - 72;
            y = cachedPose.getY() - 72;

            // === OPTIMIZED: Use radians directly, convert once ===
            double headingRad = cachedPose.getHeading();
            heading = Math.toDegrees(headingRad);
            if (heading < 0) heading += 360;

           correctWithLimelight();

            // === OPTIMIZED: Combined angle calculation ===
            double dx = xt - x;
            double dy = yt - y;
            distanceToTarget = Math.sqrt(dx * dx + dy * dy);

            double fieldAngleDeg = Math.toDegrees(Math.atan2(dy, dx));
            if (fieldAngleDeg < 0) fieldAngleDeg += 360;

            targetAngleDeg = fieldAngleDeg - heading + 180 + AngleOffset + AngleAdjust;
            targetAngleDeg = normalizeDegrees(targetAngleDeg);

            // === READ CURRENT POSITION ===
            currentServoPos = turretServo1.getPosition();
            turretAngleDeg = servoToAngle(currentServoPos);

            // === Initialize commanded angle if needed — always normalized ===
            if (!commandedAngleInitialized) {
                commandedAngle = normalizeDegrees(turretAngleDeg);
                commandedAngleInitialized = true;
            }

            // === CALCULATE ERROR ===
            double error = targetAngleDeg - commandedAngle;

            // Wrap error to the shortest path
            if (error > 180) error -= 360;
            else if (error < -180) error += 360;

            // === EARLY EXIT: Skip if within tolerance ===
            if (Math.abs(error) < 0.5) {
                lastUpdateTime = currentTime;
                return;
            }

            // === CALCULATE TIME DELTA ===
            double dt = firstRun ? MIN_LOOP_TIME : timeSinceLastUpdate;
            if (dt <= 0 || dt > 0.2) dt = MIN_LOOP_TIME;
            firstRun = false;

            // === PID CALCULATION ===
            double P_output = kP * error;

            integral += error * dt;
            if (Math.abs(error) < TOLERANCE) integral = 0;
            integral = clamp(integral, -100, 100);
            double I_output = kI * integral;

            double derivative = (error - lastError) / dt;
            double D_output = kD * derivative;

            double pidOutput = clamp(P_output + I_output + D_output, -MAX_VELOCITY, MAX_VELOCITY);

            // === UPDATE POSITION — normalize commandedAngle to prevent unbounded drift ===
            commandedAngle += pidOutput * dt;
            commandedAngle = normalizeDegrees(commandedAngle);  // FIX: keep commandedAngle bounded
            double newServoPos = angleToServo(commandedAngle);  // no need for separate normalizedAngle
            newServoPos = clamp(newServoPos, SERVO_MIN, SERVO_MAX);

            // === OPTIMIZED: Single comparison, batch servo writes ===
            if (Math.abs(newServoPos - currentServoPos) > 0.002) {
                turretServo1.setPosition(newServoPos);
                turretServo2.setPosition(newServoPos);
            }

            // === UPDATE STATE ===
            lastError = error;
            lastUpdateTime = currentTime;



        } catch (Exception e) {
            // Silent catch to prevent crashes
        }
    }

    // === OPTIMIZED: Lightweight manual mode telemetry update ===
//    private void updateManualModeTelemetry() {
//        if (PedroComponent.follower() != null) {
//            Pose currentPose = PedroComponent.follower().getPose();
//            if (currentPose != null) {
//                x = currentPose.getX() - 72;
//                y = currentPose.getY() - 72;
//                heading = Math.toDegrees(currentPose.getHeading());
//                if (heading < 0) heading += 360;
//
//                double dx = xt - x;
//                double dy = yt - y;
//                double fieldAngleDeg = Math.toDegrees(Math.atan2(dy, dx));
//                if (fieldAngleDeg < 0) fieldAngleDeg += 360;
//
//                distanceToTarget = Math.sqrt(dx * dx + dy * dy);
//                targetAngleDeg = fieldAngleDeg - heading + 180 + AngleOffset + AngleAdjust;
//                targetAngleDeg = normalizeDegrees(targetAngleDeg);
//
//                if (hardwareInitialized && turretServo1 != null) {
//                    currentServoPos = turretServo1.getPosition();
//                    turretAngleDeg = servoToAngle(currentServoPos);
//                }



    // ==================== LIMELIGHT CORRECTION ====================
    /**
     * Uses MegaTag (Pipeline 4) to:
     *   1. Correct PedroPathing odometry pose to Limelight's bot-pose estimate.
     *   2. Correct AngleAdjust to account for accumulated servo offset error.
     *
     * Call this from your OpMode at whatever cadence you want (e.g. every 500ms,
     * or triggered by a button). It is safe to call from periodic() with the
     * interval guard below — it will self-throttle.
     *
     * @return true if a correction was successfully applied, false otherwise.
     */
    public boolean correctWithLimelight() {
        lastCorrectionSucceeded = false;

        if (!limelightCorrectionEnabled)  return false;
        if (!hardwareInitialized)         return false;
        if (limelight == null)            return false;
        if (PedroComponent.follower() == null) return false;

        // Self-throttle so periodic() callers don't spam corrections
        if (limelightCorrectionTimer.seconds() - lastLimelightCorrectionTime
                < LIMELIGHT_CORRECTION_INTERVAL) {
            return false;
        }

        try {
            // ── 1. Switch to MegaTag pipeline and grab a fresh result ──────────
            limelight.pipelineSwitch(4);
            limelight.updateRobotOrientation(heading); // feed IMU heading for MegaTag2 if used

            LLResult result = limelight.getLatestResult();
            if (result == null || !result.isValid()) return false;

            // Require at least one visible AprilTag for a trustworthy fix
            if (result.getFiducialResults() == null
                    || result.getFiducialResults().isEmpty()) return false;

            Pose3D botPose3D = result.getBotpose();
            if (botPose3D == null) return false;

            // ── 2. Convert LL bot-pose to field inches (same convention as getRobotPosFromTarget) ──
            double llX = botPose3D.getPosition().y / 0.0254 + 70.625;   // inches, field coords
            double llY = -botPose3D.getPosition().x / 0.0254 + 70.625;

            double llYawDeg = botPose3D.getOrientation().getYaw(AngleUnit.DEGREES) + 90.0;
            if (llYawDeg > 360) llYawDeg -= 360;
            if (llYawDeg < 0)   llYawDeg += 360;
            double llYawRad = Math.toRadians(llYawDeg);

            // ── 3. Sanity check: reject if correction magnitude is unreasonably large ──
            double dx = llX - (x + 72); // x is stored offset by -72
            double dy = llY - (y + 72);
            double correctionMag = Math.sqrt(dx * dx + dy * dy);
            if (correctionMag > MAX_POSE_CORRECTION_INCHES) return false;

            lastPoseCorrectionMag = correctionMag;

            // ── 4. Compute the corrected target angle BEFORE updating pose ──────
            //    (so we can diff against what odometry was saying)
            double odoCenteredX = x;  // already = pose.x - 72
            double odoCenteredY = y;
            double odoDx = xt - odoCenteredX;
            double odoDy = yt - odoCenteredY;
            double odoFieldAngleDeg = Math.toDegrees(Math.atan2(odoDy, odoDx));
            if (odoFieldAngleDeg < 0) odoFieldAngleDeg += 360;
            double odoTargetAngle = normalizeDegrees(odoFieldAngleDeg - heading + 180 + AngleOffset + AngleAdjust);

            // ── 5. Commit pose correction to Pedro follower ────────────────────
            //    Pedro stores pose as raw field coords (before our -72 centering)
            Pose correctedPose = new Pose(llX, llY, llYawRad);
            PedroComponent.follower().setPose(correctedPose);

            // ── 6. Recompute target angle with LL-corrected pose ───────────────
            double llCenteredX = llX - 72;
            double llCenteredY = llY - 72;
            double llDx = xt - llCenteredX;
            double llDy = yt - llCenteredY;
            double llFieldAngleDeg = Math.toDegrees(Math.atan2(llDy, llDx));
            if (llFieldAngleDeg < 0) llFieldAngleDeg += 360;
            double llTargetAngle = normalizeDegrees(llFieldAngleDeg - llYawDeg + 180 + AngleOffset + AngleAdjust);

            // ── 7. Derive servo offset correction ─────────────────────────────
            //    If odometry was wrong, the turret has been pointed at the wrong angle.
            //    The difference tells us how far off AngleAdjust needs to shift.
            double angleCorrection = normalizeDegrees(llTargetAngle - odoTargetAngle);

            // Clamp correction to prevent a single bad reading from wildly swinging the turret
            angleCorrection = clamp(angleCorrection, -MAX_ANGLE_CORRECTION_DEG, MAX_ANGLE_CORRECTION_DEG);

            // Apply a soft blend (0.5 = half-correction per call, tune as needed)
            double blendFactor = 0.5;
            double adjustDelta = angleCorrection * blendFactor;
            AngleAdjust += adjustDelta;
            AngleAdjust  = clamp(AngleAdjust, -45, 45); // safety rails

            lastAngleCorrectionDelta = adjustDelta;

            // ── 8. Reset PID integral to prevent windup after pose jump ────────
            integral = 0;
            lastError = 0;

            // ── 9. Update diagnostics ──────────────────────────────────────────
            lastLimelightCorrectionTime = limelightCorrectionTimer.seconds();
            limelightCorrectionCount++;
            lastCorrectionSucceeded = true;

            return true;

        } catch (Exception e) {
            return false;
        }
    }

    // ── Diagnostic getters ───────────────────────────────────────────────────────
    public int LimelightCorrectionCount()  { return limelightCorrectionCount; }
    public double getLastPoseCorrectionMag()     { return lastPoseCorrectionMag; }
    public double getLastAngleCorrectionDelta()  { return lastAngleCorrectionDelta; }
    public boolean wasLastCorrectionSuccessful() { return lastCorrectionSucceeded; }

    public Pose getRobotPosFromTarget() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D robotPos = result.getBotpose();
            double angle = robotPos.getOrientation().getYaw(AngleUnit.DEGREES) + 90;
            if (angle > 360) angle -= 360;
            return new Pose((robotPos.getPosition().y / 0.0254) + 70.625, (-robotPos.getPosition().x / 0.0254) + 70.625);
        }
        return null;
    }

    // ------------------ Limelight Relocalization ------------------
    /**
     * Uses MegaTag (Pipeline 4) to snap Pedro odometry to Limelight's bot-pose.
     * Call this on a button press or at a timed interval.
     *
     * @return true if relocalization was applied, false if no valid result.
     */
    public boolean relocalize() {
        if (!hardwareInitialized) return false;
        if (limelight == null) return false;
        if (PedroComponent.follower() == null) return false;

        try {
            limelight.pipelineSwitch(4);
            limelight.updateRobotOrientation(heading); // feed current heading to MegaTag

            LLResult result = limelight.getLatestResult();
            if (result == null || !result.isValid()) return false;

            // Require at least one visible AprilTag
            if (result.getFiducialResults() == null
                    || result.getFiducialResults().isEmpty()) return false;

            // Use the existing conversion method
            Pose llPose = getRobotPosFromTarget();
            if (llPose == null) return false;

            // Sanity check — reject if LL says we jumped too far from current odo
            double currentX = PedroComponent.follower().getPose().getX();
            double currentY = PedroComponent.follower().getPose().getY();
            double dx = llPose.getX() - currentX;
            double dy = llPose.getY() - currentY;
            double correctionMag = Math.sqrt(dx * dx + dy * dy);
            if (correctionMag > MAX_POSE_CORRECTION_INCHES) return false;

            // Preserve current heading from Pedro since getRobotPosFromTarget() doesn't return one
            double currentHeading = PedroComponent.follower().getPose().getHeading();
            Pose correctedPose = new Pose(llPose.getX(), llPose.getY(), currentHeading);
            PedroComponent.follower().setPose(correctedPose);

            // Reset PID to avoid windup after the pose snap
            integral = 0;
            lastError = 0;

            return true;

        } catch (Exception e) {
            return false;
        }
    }

    // ------------------ Helper Functions ------------------
    public void turnRight() {
        AngleAdjust += 2;
    }

    public void turnLeft() {
        AngleAdjust -= 2;
    }

    private double angleToServo(double angleDeg) {
        angleDeg = normalizeDegrees(angleDeg);
        double pos = 1 - ((angleDeg + 180) / 340);
        return clamp(pos, SERVO_MIN, SERVO_MAX);
    }

    private double servoToAngle(double servoPos) {
        double angle = 340 * (1.0 - servoPos) - 180.0;
        return normalizeDegrees(angle);
    }

    /**
     * OPTIMIZED: Faster normalization without modulo
     */
    public double normalizeDegrees(double angle) {
        if (angle > 180) {
            angle -= 340;
            if (angle > 180) angle -= 340;
        } else if (angle < -180) {
            angle += 340;
            if (angle < -180) angle += 340;
        }
        return angle;
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    // ------------------ Getters ------------------
    public double getX() { return x; }
    public double getY() { return y; }
    public double getHeading() { return heading; }
    public double getTargetAngleDeg() { return targetAngleDeg; }
    public double getTurretAngleDeg() { return turretAngleDeg; }
    public double getDistanceToTarget() { return distanceToTarget; }
    public double getLastError() { return lastError; }
    public double getIntegral() { return integral; }
    public double getKp() { return kP; }
    public int getSkippedLoops() { return skippedLoops; }
    public double getLoopTime() { return loopTimer.seconds() - lastUpdateTime; }
    public boolean isManualMode() { return manualMode; }
    public double getManualPosition() { return manualPosition; }
    public double getCommandedAngle() { return commandedAngle; }
}