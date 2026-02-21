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

    // ------------------ Hardware ------------------
    private Servo turretServo1;
    private Servo turretServo2;
    private Limelight3A limelight;

    // ------------------ Robot Pose (odometry fallback) ------------------
    private double x = 0;
    private double y = 0;
    private double heading = 0;

    public double ManualAngleAdjust = 0;

    // ------------------ Target ------------------
    public static double xt = 130;
    public static double yt = 130;
    double AngleOffset = -30;

    // ------------------ Turret state ------------------
    private double targetAngleDeg = 0;
    private double turretAngleDeg = 0;
    private double distanceToTarget = 0;
    private double currentServoPos = 0;

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
    public static double MAX_VELOCITY = 1100;
    public static double TOLERANCE = 0.1;
    public double AngleAdjust = 0;

    // ========== PID STATE ==========
    private double lastError = 0;
    private double integral = 0;
    private double lastUpdateTime = 0;
    private boolean firstRun = true;
    private double commandedAngle = 0;
    private boolean commandedAngleInitialized = false;

    // ========== RATE LIMITING ==========
    private ElapsedTime loopTimer = new ElapsedTime();
    private static final double MIN_LOOP_TIME = 0.010;
    private int skippedLoops = 0;

    // ========== LIMELIGHT ==========
    // Limelight is PRIMARY. Odometry is fallback only.
    public static int    TARGET_TAG_ID     = -1;   // set per alliance in setAlliance()
    public static double TX_OFFSET         = -1.0; // tune: negative = shift left
    public static double TX_SIGN_FLIP      = 1.0;  // flip to -1.0 if turret corrects wrong way
    public static double TX_TOLERANCE_DEG  = 0.5;  // dead-band — inside this, hold position

    // Whether Limelight currently has a valid lock on the target tag
    private boolean limelightLocked = false;

    // Limelight orientation update rate limiting (separate from main loop)
    private ElapsedTime llOrientationTimer = new ElapsedTime();
    private static final double LL_ORIENTATION_INTERVAL = 0.1; // update heading 10x/sec max

    // ========== DIAGNOSTICS ==========
    private double lastPoseCorrectionMag = 0;
    private int    limelightCorrectionCount = 0;
    public static double MAX_POSE_CORRECTION_INCHES = 12.0;

    // ========== PERFORMANCE ==========
    private Pose cachedPose = null;

    private TurretOdoAi() {}

    // ------------------ Alliance Setup ------------------
    public void setAlliance(String alliance) {
        if (alliance.equals("blue")) {
            AngleOffset = -30 + 90;
            TARGET_TAG_ID = 20;
            if (limelight != null) limelight.pipelineSwitch(5);
        } else if (alliance.equals("red")) {
            AngleOffset = -36;
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

            commandedAngle = normalizeDegrees(servoToAngle(0.25));
            commandedAngleInitialized = true;

            loopTimer.reset();
            llOrientationTimer.reset();
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
            commandedAngle = normalizeDegrees(servoToAngle(manualPosition));
        }
    }

    // ------------------ Main Loop ------------------
    @Override
    public void periodic() {
        if (!hardwareInitialized) return;

        double currentTime = loopTimer.seconds();
        double timeSinceLastUpdate = currentTime - lastUpdateTime;
        if (timeSinceLastUpdate < MIN_LOOP_TIME) {
            skippedLoops++;
            return;
        }

        if (PedroComponent.follower() == null) return;

        try {
            // === 1. Update odometry pose (always needed for heading + fallback) ===
            cachedPose = PedroComponent.follower().getPose();
            if (cachedPose == null) return;

            x = cachedPose.getX() - 72;
            y = cachedPose.getY() - 72;
            heading = Math.toDegrees(cachedPose.getHeading());
            if (heading < 0) heading += 360;

            // === 2. Feed heading to Limelight at a safe rate (no pipelineSwitch here!) ===
            if (limelight != null && llOrientationTimer.seconds() > LL_ORIENTATION_INTERVAL) {
                limelight.updateRobotOrientation(heading);
                llOrientationTimer.reset();
            }

            // === 3. Try Limelight FIRST — it is the primary source of targetAngleDeg ===
            limelightLocked = false;
            if (limelight != null) {
                LLResult result = limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                    if (fiducials != null && !fiducials.isEmpty()) {

                        LLResultTypes.FiducialResult targetFiducial = null;
                        for (LLResultTypes.FiducialResult f : fiducials) {
                            if (TARGET_TAG_ID < 0 || f.getFiducialId() == TARGET_TAG_ID) {
                                targetFiducial = f;
                                break;
                            }
                        }

                        if (targetFiducial != null) {
                            // === LIMELIGHT IS PRIMARY ===
                            // tx = how many degrees the tag is left/right of the camera crosshair.
                            // We add that directly to the current turret angle to get the true target.
                            double tx = targetFiducial.getTargetXDegrees() + TX_OFFSET;

                            currentServoPos = turretServo1.getPosition();
                            turretAngleDeg  = servoToAngle(currentServoPos);

                            // Target = where turret is NOW, shifted by tx to centre on tag
                            targetAngleDeg = normalizeDegrees(turretAngleDeg + TX_SIGN_FLIP * tx + AngleAdjust + ManualAngleAdjust);                            limelightLocked = true;

                            // Also opportunistically correct Pedro pose from bot-pose
                            tryPoseCorrection(result);
                        }
                    }
                }
            }

            // === 4. Fallback to odometry ONLY if Limelight has no lock ===
            if (!limelightLocked) {
                double dx = xt - x;
                double dy = yt - y;
                distanceToTarget = Math.sqrt(dx * dx + dy * dy);

                double fieldAngleDeg = Math.toDegrees(Math.atan2(dy, dx));
                if (fieldAngleDeg < 0) fieldAngleDeg += 360;

                targetAngleDeg = normalizeDegrees(fieldAngleDeg - heading + 180 + AngleOffset + AngleAdjust + ManualAngleAdjust);            }

            // === 5. Read current servo position ===
            currentServoPos = turretServo1.getPosition();
            turretAngleDeg  = servoToAngle(currentServoPos);

            if (!commandedAngleInitialized) {
                commandedAngle = normalizeDegrees(turretAngleDeg);
                commandedAngleInitialized = true;
            }

            // === 6. PID toward targetAngleDeg ===
            double error = targetAngleDeg - commandedAngle;
            if (error >  180) error -= 360;
            if (error < -180) error += 360;

            // Skip PID update if locked on and within dead-band
            if (limelightLocked && Math.abs(error) < TX_TOLERANCE_DEG) {
                lastUpdateTime = currentTime;
                return;
            }
            // Odometry tolerance
            if (!limelightLocked && Math.abs(error) < 0.5) {
                lastUpdateTime = currentTime;
                return;
            }

            double dt = firstRun ? MIN_LOOP_TIME : timeSinceLastUpdate;
            if (dt <= 0 || dt > 0.2) dt = MIN_LOOP_TIME;
            firstRun = false;

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
            // Silent catch — never crash the loop
        }
    }

    // ------------------ Pose Correction (opportunistic, no pipeline switch) ------------------
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
            // Ignore pose correction failures
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
        if (angle >  180) { angle -= 360; if (angle >  180) angle -= 360; }
        if (angle < -180) { angle += 360; if (angle < -180) angle += 360; }
        return angle;
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    // ------------------ Getters ------------------
    public double  getX()                 { return x; }
    public double  getY()                 { return y; }
    public double  getHeading()           { return heading; }
    public double  getTargetAngleDeg()    { return targetAngleDeg; }
    public double  getTurretAngleDeg()    { return turretAngleDeg; }
    public double  getDistanceToTarget()  { return distanceToTarget; }
    public double  getLastError()         { return lastError; }
    public double  getIntegral()          { return integral; }
    public double  getKp()                { return kP; }
    public int     getSkippedLoops()      { return skippedLoops; }
    public double  getLoopTime()          { return loopTimer.seconds() - lastUpdateTime; }
    public boolean isManualMode()         { return manualMode; }
    public double  getManualPosition()    { return manualPosition; }
    public double  getCommandedAngle()    { return commandedAngle; }
    public boolean isLimelightLocked()    { return limelightLocked; }
    public int     getLimelightCorrectionCount() { return limelightCorrectionCount; }
    public double  getLastPoseCorrectionMag()    { return lastPoseCorrectionMag; }
}