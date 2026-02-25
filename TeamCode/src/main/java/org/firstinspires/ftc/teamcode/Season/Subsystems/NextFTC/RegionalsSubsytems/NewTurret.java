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

/**
 * NewTurret — Combined turret controller for FTC.
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * ARCHITECTURE — Three-layer control, two states
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *   LAYER 1 — HEADING FEEDFORWARD (every loop, both states)
 *   ─────────────────────────────────────────────────────────────────────
 *   The robot just turned Δθ degrees. The turret MUST counter-rotate by
 *   Δθ to maintain the same field-relative bearing. This happens first,
 *   before any feedback loop runs, so it is PROACTIVE — the turret moves
 *   in lockstep with the chassis with zero lag.
 *
 *   This layer runs regardless of whether the tag is visible, regardless
 *   of which state the machine is in. It is the reason the turret stays
 *   on goal during fast turns even when Limelight momentarily loses the tag.
 *
 *   LAYER 2 — SEEKING state (odometry bearing PID)
 *   ─────────────────────────────────────────────────────────────────────
 *   Active when the tag is not visible or the turret is far off-bearing.
 *   Computes the field-relative bearing from odometry, finds the residual
 *   error that feedforward didn't cover (heading lag, odometry drift), and
 *   drives it to zero with a PID. Integrates in servo space to avoid the
 *   ±180° wrap discontinuity that plagued TurretOdoAi.
 *
 *   → Transitions to FINE_ALIGN when:
 *       • Tag visible for LOCK_CONFIRM_FRAMES consecutive loops, AND
 *       • |odometry bearing error| ≤ FINE_ALIGN_ENTRY_DEG
 *
 *   LAYER 3 — FINE_ALIGN state (2D Limelight tx P control)
 *   ─────────────────────────────────────────────────────────────────────
 *   Active when the tag is in frame and we're pointing close enough.
 *   Feedforward still runs first. tx error then corrects whatever small
 *   residual remains — immune to odometry drift, gives sub-degree precision.
 *   Mirrors BlueLL / RedLL logic but for position-mode servos.
 *
 *   → Transitions back to SEEKING when:
 *       • Tag absent for LOCK_LOST_FRAMES consecutive loops
 *       (feedforward keeps the turret on bearing during the gap)
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * WHY FEEDFORWARD MUST RUN IN BOTH STATES
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *   Without FF in FINE_ALIGN: during a turn the robot rotates but the turret
 *   holds position. The tag walks out of frame → SEEKING fallback → slow
 *   re-acquisition. With FF in FINE_ALIGN: the turret counter-rotates with
 *   the chassis instantly; tx sees only a small residual error and corrects
 *   it quickly without ever losing the tag.
 *
 *   Without FF in SEEKING: the PID reacts only AFTER error builds up (one
 *   full loop of lag). With FF in SEEKING: the turret moves in the same loop
 *   the robot turns; PID only handles residual. Much faster tracking.
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * WRAPPING-SAFE INTEGRATION
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *   All servo movement is computed as a DELTA to currentServoPos, which is
 *   a continuous 0.0–1.0 float. It is never converted back to an angle and
 *   re-normalized, so there is no ±180° snap discontinuity (the bug in
 *   TurretOdoAi where commandedAngle jumped from −181° → +179°, producing
 *   a large servo position discontinuity that slammed the turret to its stop).
 *
 *   Error is computed in angle space (normalizeDegrees → always ±180°) to
 *   find the shortest path. The result is converted to a servo delta and
 *   accumulated. The servo value is just clamped, never normalized.
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * TUNING ORDER
 * ═══════════════════════════════════════════════════════════════════════════
 *   1. Set FF_GAIN = 1.0, FF_SIGN = +1.0. Watch turret during robot turns.
 *      If turret moves AWAY from target during turns → FF_SIGN = -1.0.
 *      If turret over-shoots during turns → reduce FF_GAIN (try 0.8).
 *      Goal: turret stays locked on goal even while robot spins fast.
 *
 *   2. FINE_ALIGN_ENTRY_DEG (default 12°): widen if tag leaves frame before
 *      FINE_ALIGN kicks in; narrow to spend more time in SEEKING.
 *
 *   3. CAMERA_MOUNT_OFFSET_DEG: adjust until getLastTx() ≈ 0 on target.
 *
 *   4. TX_SIGN_FLIP: flip to −1.0 if FINE_ALIGN moves turret AWAY from tag.
 *
 *   5. FINE_kP / FINE_MAX_STEP: tune fine-align speed vs. overshoot.
 *
 *   6. kP_seek: SEEKING swing speed. Higher = faster, more overshoot.
 */
public class NewTurret implements Subsystem {

    public static final NewTurret INSTANCE = new NewTurret();

    // ─────────────────────────────────────────────────────────────────────
    // STATE MACHINE
    // ─────────────────────────────────────────────────────────────────────
    public enum TurretState { SEEKING, FINE_ALIGN }
    private TurretState state = TurretState.SEEKING;

    /**
     * Odometry bearing error threshold (degrees) for SEEKING → FINE_ALIGN.
     * Prevents handing off to tx control while the tag is still off-screen.
     * Widen if the tag escapes frame before handoff; narrow for earlier lock.
     */
    public static double FINE_ALIGN_ENTRY_DEG = 12.0;

    /**
     * Odometry bearing error threshold (degrees) that forces an IMMEDIATE
     * return to SEEKING from FINE_ALIGN, even when the tag is still visible.
     *
     * WHY: During a large robot turn the heading feedforward moves the turret
     * most of the way, but if the residual odoError grows past this threshold
     * it means the turret is significantly off-bearing and the 2D tx loop is
     * too slow to catch up.  Handing back to the faster odometry PID prevents
     * the tag from walking out of frame on aggressive manoeuvres.
     *
     * Rule of thumb: set to roughly half your Limelight's horizontal FOV.
     * Default 5° works well for a ~60° FOV camera (tag stays visible up to ~30°).
     * Increase if SEEKING kicks in too aggressively on small corrections.
     */
    public static double ODO_OVERRIDE_DEG = 5.0;

    /** Consecutive visible frames required before entering FINE_ALIGN. */
    public static int LOCK_CONFIRM_FRAMES = 1;

    /** Consecutive missed frames before falling back to SEEKING. */
    public static int LOCK_LOST_FRAMES = 10;

    private int consecutiveDetections = 0;
    private int consecutiveMisses     = 0;

    // ─────────────────────────────────────────────────────────────────────
    // HARDWARE
    // ─────────────────────────────────────────────────────────────────────
    private Servo       turretServo1;
    private Servo       turretServo2;
    private Limelight3A limelight;
    public  boolean     hardwareInitialized = false;
    public  boolean     manualMode          = false;

    // ─────────────────────────────────────────────────────────────────────
    // ODOMETRY STATE
    // ─────────────────────────────────────────────────────────────────────
    private double x           = 0;
    private double y           = 0;
    private double heading     = 0;  // degrees, 0–360
    private double lastHeading = 0;
    private double headingRate = 0;  // degrees/sec — positive = CCW in standard convention

    // ─────────────────────────────────────────────────────────────────────
    // FIELD TARGET (odometry bearing fallback)
    // ─────────────────────────────────────────────────────────────────────
    public static double xt = 130;
    public static double yt = 130;

    public  static int    TARGET_TAG_ID     = -1;
    private        double AngleOffset       = -36;
    public         double AngleAdjust       = 0;
    public         double ManualAngleAdjust = 0;

    private double targetAngleDeg   = 0;
    private double turretAngleDeg   = 0;
    private double distanceToTarget = 0;

    // ─────────────────────────────────────────────────────────────────────
    // SERVO POSITION — single source of truth
    //
    // NEVER replace with servo.getPosition(). Always use this variable.
    //
    // Mapping (inverted — higher angle → lower servo position):
    //   angle = 350·(1−pos) − 180
    //   pos   = 1 − (angle+180) / 350
    //   pos = 0.0  ↔  angle = +170°
    //   pos = 0.5  ↔  angle ≈   −5°
    //   pos = 1.0  ↔  angle = −180°
    // ─────────────────────────────────────────────────────────────────────
    private double currentServoPos = 0.25;
    public static double SERVO_MIN = 0.0;
    public static double SERVO_MAX = 1.0;

    // 350° over 0.0–1.0 → degrees per servo unit (for delta conversion)
    private static final double DEG_PER_SERVO_UNIT = 350.0;

    // ═════════════════════════════════════════════════════════════════════
    // LAYER 1 — HEADING RATE FEEDFORWARD
    //
    // Applied EVERY loop BEFORE any feedback correction.
    // When the robot rotates Δθ degrees in one loop, the turret must
    // counter-rotate by the same amount to hold its field-relative bearing.
    //
    // MATH:
    //   headingDelta (deg) = headingRate (deg/s) × dt (s)
    //   Δpos = −(headingDelta × FF_SIGN × FF_GAIN) / DEG_PER_SERVO_UNIT
    //   The negative sign is from the inverted servo mapping:
    //     Δangle = −350·Δpos  →  Δpos = −Δangle/350
    //   Robot CW (positive headingDelta in our convention) → turret must
    //   also swing CW to stay on goal → angle increases → pos decreases.
    //   FF_SIGN handles the physical case where this is reversed.
    //
    // TUNING:
    //   FF_SIGN (+1.0 or −1.0): flip if turret moves AWAY from goal on turns.
    //   FF_GAIN (0.0–1.5): 1.0 = full compensation. Reduce if turret
    //     over-leads turns; increase if it still lags significantly after
    //     adjusting FF_SIGN. Start at 1.0 and adjust in 0.1 increments.
    // ═════════════════════════════════════════════════════════════════════
    public static double FF_SIGN = 1.0;
    public static double FF_GAIN = 1.0;

    // ─────────────────────────────────────────────────────────────────────
    // LAYER 2 — SEEKING PID (odometry bearing, servo-space integration)
    // ─────────────────────────────────────────────────────────────────────
    public static double kP_seek      = 40.0;
    public static double kI_seek      = 0.0;
    public static double kD_seek      = 0.0;
    public static double MAX_VELOCITY = 1100.0; // degrees/sec cap on PID output
    public static double TOLERANCE    = 0.25;   // degrees — skip update below this

    private double  seekLastError = 0;
    private double  seekIntegral  = 0;
    private boolean firstRun      = true;

    // ─────────────────────────────────────────────────────────────────────
    // LAYER 3 — FINE ALIGN (2D Limelight tx, position-servo step control)
    //
    // Mirrors BlueLL / RedLL alignWithKpAndOffset() but for position servos:
    //   CRServo.setPower(p) held continuously
    //   → position servo steps by (BASE_STEP + kP×|err|) per loop.
    // ─────────────────────────────────────────────────────────────────────

    /** tx dead-band (degrees). No correction inside this window. */
    public static double FINE_ALIGN_THRESHOLD_DEG = 0.1;

    /**
     * Minimum position step per loop when outside the dead-band.
     * Mirrors BASE_POWER = 0.03 in BlueLL/RedLL.
     */
    public static double FINE_BASE_STEP = 0.006;

    /**
     * Proportional gain: step += FINE_kP × |tx_error_deg|
     * Mirrors kP_CLOSE = 0.01 in BlueLL/RedLL, scaled to position units.
     * Higher = faster centering, lower = less overshoot.
     */
    public static double FINE_kP = 0.05;

    /**
     * Maximum position step per loop.
     * Mirrors MAX_POWER = 0.8 in BlueLL/RedLL — prevents slamming.
     * At 100 Hz loop rate, 0.04 step/loop × 350 deg/unit = 14 deg/loop max.
     */
    public static double FINE_MAX_STEP = 0.04;

    /**
     * Physical camera offset from barrel centerline (degrees).
     * Adjust until getLastTx() ≈ 0 when barrel is on target.
     */
    public static double CAMERA_MOUNT_OFFSET_DEG = 7.0;

    /** Fine crosshair bias trim (degrees). Adjust after CAMERA_MOUNT_OFFSET_DEG. */
    public static double TX_OFFSET = 0.0;

    /**
     * +1.0 or −1.0.
     * Flip if FINE_ALIGN moves the turret AWAY from the tag instead of toward it.
     */
    public static double TX_SIGN_FLIP = 1.0;

    /**
     * Per-alliance tx offset (degrees). Set via setAlliance(); override via
     * setAllianceTxOffset(). Mirrors the per-method offsets in BlueLL/RedLL.
     */
    private double allianceTxOffset = 0.0;

    // ─────────────────────────────────────────────────────────────────────
    // TIMING
    // ─────────────────────────────────────────────────────────────────────
    private final ElapsedTime loopTimer         = new ElapsedTime();
    private static final double MIN_LOOP_TIME   = 0.010;
    private double lastUpdateTime               = 0;
    private int    skippedLoops                 = 0;

    private final ElapsedTime llOrientationTimer        = new ElapsedTime();
    private static final double LL_ORIENTATION_INTERVAL = 0.033; // ~30 Hz — faster pose updates

    // ─────────────────────────────────────────────────────────────────────
    // DIAGNOSTICS
    // ─────────────────────────────────────────────────────────────────────
    private double lastTx                   = 0;
    private double lastFineServoDelta       = 0;
    private double lastFFServoDelta         = 0;  // feedforward contribution this loop
    private double lastOdoError             = 0;
    private double lastPoseCorrectionMag    = 0;
    private int    limelightCorrectionCount = 0;
    public static double MAX_POSE_CORRECTION_INCHES = 12.0;

    /**
     * Accumulated tx-based correction to odometry bearing (degrees).
     *
     * WHY THIS EXISTS:
     *   AngleOffset was tuned at one robot position/heading. At different
     *   field positions odometry drift means the computed bearing is slightly
     *   off. When FINE_ALIGN has the tag centered (tx ≈ 0), the turret IS
     *   pointing at the goal, so we know the true bearing. We compare that
     *   to what odometry computed and add the gap to this correction term,
     *   which is then added to targetAngleDeg every loop in SEEKING.
     *   Result: each FINE_ALIGN lock automatically recalibrates the odometry
     *   bearing for the next time SEEKING runs — no manual retuning needed.
     *
     * RESET: call resetOdoCorrection() to zero it out (e.g. at match start).
     * BOUNDS: clamped to ±ODO_CORRECTION_MAX_DEG to reject bad measurements.
     */
    private double odoAngleCorrection = 0.0;
    public static double ODO_CORRECTION_MAX_DEG  = 30.0; // reject corrections outside this
    public static double ODO_CORRECTION_ALPHA    = 0.15; // low-pass weight (0=no update, 1=instant)

    // Software-tracked commanded angle — kept in sync so SEEKING always
    // resumes from the servo's current physical position after FINE_ALIGN.
    private double commandedAngle = 0;

    private Pose cachedPose = null;

    // ─────────────────────────────────────────────────────────────────────
    // SINGLETON
    // ─────────────────────────────────────────────────────────────────────
    private NewTurret() {}

    // ═════════════════════════════════════════════════════════════════════
    // ALLIANCE SETUP
    // ═════════════════════════════════════════════════════════════════════
    public void setAlliance(String alliance) {
        if (alliance.equalsIgnoreCase("blue")) {
            AngleOffset      = -36 + 90 - 45; // tuned value from TurretOdoAi
            TARGET_TAG_ID    = 20;
            allianceTxOffset = 0.0;
            if (limelight != null) limelight.pipelineSwitch(0);
        } else if (alliance.equalsIgnoreCase("red")) {
            AngleOffset      = -45 - 45;       // tuned value from TurretOdoAi
            xt               = 130;
            yt               = 130;
            TARGET_TAG_ID    = 24;
            allianceTxOffset = 0.0;
            if (limelight != null) limelight.pipelineSwitch(1);
        }
    }

    /**
     * Override the per-alliance tx offset at runtime.
     * e.g. setAllianceTxOffset(-4) mirrors BlueLL.cycleAlign(offset=−4).
     */
    public void setAllianceTxOffset(double offsetDeg) {
        allianceTxOffset = offsetDeg;
    }

    // ═════════════════════════════════════════════════════════════════════
    // INIT
    // ═════════════════════════════════════════════════════════════════════
    public void init(HardwareMap hardwareMap) {
        try {
            turretServo1 = hardwareMap.get(Servo.class, "turretServo1");
            turretServo2 = hardwareMap.get(Servo.class, "turretServo2");
            limelight    = hardwareMap.get(Limelight3A.class, "Limelight");

            currentServoPos = 0.25;
            turretServo1.setPosition(currentServoPos);
            turretServo2.setPosition(currentServoPos);

            loopTimer.reset();
            llOrientationTimer.reset();
            lastUpdateTime = loopTimer.seconds();
            firstRun       = true;

            state                 = TurretState.SEEKING;
            consecutiveDetections = 0;
            consecutiveMisses     = 0;
            seekIntegral          = 0;
            seekLastError         = 0;
            odoAngleCorrection    = 0.0;
            commandedAngle        = normalizeDegrees(servoToAngle(currentServoPos));

            hardwareInitialized = true;
        } catch (Exception e) {
            hardwareInitialized = false;
        }
    }

    // ═════════════════════════════════════════════════════════════════════
    // MAIN LOOP
    // ═════════════════════════════════════════════════════════════════════
    @Override
    public void periodic() {
        if (!hardwareInitialized) return;

        double currentTime         = loopTimer.seconds();
        double timeSinceLastUpdate = currentTime - lastUpdateTime;
        if (timeSinceLastUpdate < MIN_LOOP_TIME) { skippedLoops++; return; }
        if (PedroComponent.follower() == null) return;

        try {
            double dt = timeSinceLastUpdate;
            if (firstRun || dt <= 0 || dt > 0.2) dt = MIN_LOOP_TIME;

            // ════════════════════════════════════════════════════════════
            // STEP 1 — Odometry pose + heading rate
            // ════════════════════════════════════════════════════════════
            cachedPose = PedroComponent.follower().getPose();
            if (cachedPose == null) return;

            x = cachedPose.getX() - 72;
            y = cachedPose.getY() - 72;

            double newHeading = Math.toDegrees(cachedPose.getHeading());
            if (newHeading < 0) newHeading += 360;

            // Heading delta this loop — wrap-aware shortest-path difference
            double headingDelta = newHeading - lastHeading;
            if (headingDelta >  180) headingDelta -= 360;
            if (headingDelta < -180) headingDelta += 360;

            // headingRate (deg/sec) — for diagnostics and feedforward magnitude
            headingRate = headingDelta / dt;

            lastHeading = newHeading;
            heading     = newHeading;

            // ════════════════════════════════════════════════════════════
            // STEP 2 — Feed heading to Limelight (rate-limited)
            // ════════════════════════════════════════════════════════════
            if (limelight != null && llOrientationTimer.seconds() > LL_ORIENTATION_INTERVAL) {
                limelight.updateRobotOrientation(heading);
                llOrientationTimer.reset();
            }

            // ════════════════════════════════════════════════════════════
            // STEP 3 — Compute odometry bearing to field target
            // ════════════════════════════════════════════════════════════
            double dx = xt - x;
            double dy = yt - y;
            distanceToTarget = Math.sqrt(dx * dx + dy * dy);

            double fieldAngleDeg = Math.toDegrees(Math.atan2(dy, dx));
            if (fieldAngleDeg < 0) fieldAngleDeg += 360;

            targetAngleDeg = normalizeDegrees(
                    fieldAngleDeg - heading + 180 + AngleOffset + AngleAdjust + ManualAngleAdjust
                            + odoAngleCorrection);

            // ════════════════════════════════════════════════════════════
            // STEP 4 — Poll Limelight
            // ════════════════════════════════════════════════════════════
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

            // ════════════════════════════════════════════════════════════
            // STEP 5 — Current turret angle + odometry bearing error
            // ════════════════════════════════════════════════════════════
            turretAngleDeg = servoToAngle(currentServoPos);

            // Shortest-path error between where we point and where we should point.
            // Normalized to ±180° — always takes the most efficient route.
            double odoError = targetAngleDeg - turretAngleDeg;
            if (odoError >  180) odoError -= 360;
            if (odoError < -180) odoError += 360;
            lastOdoError = odoError;

            // ════════════════════════════════════════════════════════════
            // STEP 6 — State machine transitions
            //
            // Three paths:
            //   A) SEEKING → FINE_ALIGN: tag confirmed in frame AND
            //      odometry says we're close enough (odoError ≤ FINE_ALIGN_ENTRY_DEG).
            //
            //   B) FINE_ALIGN → SEEKING (odo override): odoError has grown
            //      past ODO_OVERRIDE_DEG. This fires IMMEDIATELY — no frame
            //      count needed — because a large bearing error means a fast
            //      turn has outpaced the tx loop and odometry must take over
            //      before the tag walks out of frame. Tag visibility is
            //      irrelevant here: even if the tag is still visible, the
            //      odometry PID is faster at catching up to big errors.
            //
            //   C) FINE_ALIGN → SEEKING (tag lost): tag absent for
            //      LOCK_LOST_FRAMES. Feedforward keeps the turret on bearing
            //      during the gap; odometry corrects any accumulated error.
            // ════════════════════════════════════════════════════════════
            if (targetFiducial != null) {
                consecutiveDetections++;
                consecutiveMisses = 0;

                if (state == TurretState.SEEKING
                        && consecutiveDetections >= LOCK_CONFIRM_FRAMES
                        && Math.abs(odoError) <= FINE_ALIGN_ENTRY_DEG) {
                    // Path A — hand off to Limelight tx control
                    state         = TurretState.FINE_ALIGN;
                    seekIntegral  = 0;
                    seekLastError = 0;
                }
            } else {
                consecutiveMisses++;
                consecutiveDetections = 0;

                if (state == TurretState.FINE_ALIGN && consecutiveMisses >= LOCK_LOST_FRAMES) {
                    // Path C — tag genuinely gone, fall back to odometry.
                    // Re-sync commandedAngle to current servo position so SEEKING
                    // resumes from exactly where FINE_ALIGN left the turret — no jump.
                    state         = TurretState.SEEKING;
                    seekIntegral  = 0;
                    seekLastError = 0;
                    commandedAngle = normalizeDegrees(servoToAngle(currentServoPos));
                }
            }

            // Path B — odo override (checked every loop regardless of tag visibility)
            // Large bearing error means a big turn just happened; odometry is faster.
            if (state == TurretState.FINE_ALIGN && Math.abs(odoError) > ODO_OVERRIDE_DEG) {
                state         = TurretState.SEEKING;
                seekIntegral  = 0;
                seekLastError = 0;
                // Re-sync commandedAngle here too so PID starts from current position
                commandedAngle = normalizeDegrees(servoToAngle(currentServoPos));
            }

            // ════════════════════════════════════════════════════════════
            // STEP 7 — LAYER 1: HEADING FEEDFORWARD (ALWAYS, both states)
            //
            // The robot just rotated headingDelta degrees in this loop.
            // The turret must counter-rotate by the same amount so it
            // maintains its field-relative bearing with zero lag.
            //
            // This runs BEFORE any feedback correction so it is proactive —
            // the turret moves in the same milliseconds the robot turns,
            // not one loop later. Feedback (SEEKING PID or FINE_ALIGN tx)
            // then corrects only the small residual the FF didn't cover.
            //
            // Why this keeps the tag in frame during turns:
            //   Without FF: robot turns → turret lags → tag walks off screen
            //               → SEEKING fallback → slow re-acquisition.
            //   With FF:    robot turns → turret moves instantly with it
            //               → tag stays near crosshair at all times.
            //
            // Derivation:
            //   angle = 350·(1−pos) − 180
            //   Δangle = −350·Δpos
            //   Δpos   = −Δangle / 350
            //   To cancel robot rotation: Δangle_turret = headingDelta
            //   ∴ Δpos = −headingDelta / 350
            //   FF_SIGN flips direction if the servo is oriented the other way.
            //   FF_GAIN scales the compensation (1.0 = perfect, tune from there).
            // ════════════════════════════════════════════════════════════
            double ffServoDelta = -(headingDelta * FF_SIGN * FF_GAIN) / DEG_PER_SERVO_UNIT;
            lastFFServoDelta = ffServoDelta;
            currentServoPos  = clamp(currentServoPos + ffServoDelta, SERVO_MIN, SERVO_MAX);
            // Note: DO NOT call setServos() here — wait until all corrections
            // are accumulated so the servo is commanded only once per loop.

            // ════════════════════════════════════════════════════════════
            // STEP 8A — FINE_ALIGN: tx proportional correction (residual)
            //
            // Feedforward already ran above. The tx controller handles only
            // the small residual error left over — camera mount offset,
            // slight FF imprecision, and odometry drift.
            //
            // netError sign:
            //   netError > 0: tag RIGHT of crosshair → step negative
            //                 (decrease pos → turret turns right)
            //   netError < 0: tag LEFT  of crosshair → step positive
            //                 (increase pos → turret turns left)
            //
            // Tag absent (occlusion, passing obstacle): FF already ran,
            // so the turret is tracking the robot's heading change.
            // No additional tx correction — hold the FF-compensated position.
            // SEEKING resumes after LOCK_LOST_FRAMES if tag stays gone.
            // ════════════════════════════════════════════════════════════
            if (state == TurretState.FINE_ALIGN) {
                if (targetFiducial != null) {
                    double txDeg = targetFiducial.getTargetXDegrees();
                    lastTx = txDeg;

                    double netError = TX_SIGN_FLIP
                            * (txDeg + TX_OFFSET + allianceTxOffset - CAMERA_MOUNT_OFFSET_DEG);

                    if (Math.abs(netError) > FINE_ALIGN_THRESHOLD_DEG) {
                        // Proportional step, floored by BASE_STEP, capped by MAX_STEP
                        double step = FINE_BASE_STEP + FINE_kP * Math.abs(netError);
                        step = Math.min(step, FINE_MAX_STEP);

                        // Apply in the direction that reduces netError
                        double servoDelta = (netError > 0) ? -step : step;
                        lastFineServoDelta  = servoDelta;
                        currentServoPos = clamp(currentServoPos + servoDelta, SERVO_MIN, SERVO_MAX);
                    } else {
                        lastFineServoDelta = 0;

                        // ── TX-BASED BEARING CORRECTION ───────────────────────
                        // The tag is centered (tx within dead-band). This means
                        // the turret is pointing at the goal RIGHT NOW. We know:
                        //   true bearing = servoToAngle(currentServoPos)
                        //   odo bearing  = targetAngleDeg  (already includes
                        //                  AngleOffset + AngleAdjust + odoAngleCorrection)
                        //
                        // Any gap is accumulated odometry error. Fold it into
                        // odoAngleCorrection via a low-pass filter so SEEKING is
                        // more accurate the next time the tag is lost.
                        //
                        // Low-pass (alpha): small alpha = slow drift correction,
                        // large alpha = fast but noisier. 0.15 is a good start.
                        double trueBearing = normalizeDegrees(servoToAngle(currentServoPos));
                        double bearingGap  = trueBearing - targetAngleDeg;
                        if (bearingGap >  180) bearingGap -= 360;
                        if (bearingGap < -180) bearingGap += 360;

                        // Only accept corrections within the sanity bound
                        if (Math.abs(bearingGap + odoAngleCorrection) < ODO_CORRECTION_MAX_DEG) {
                            odoAngleCorrection += ODO_CORRECTION_ALPHA * bearingGap;
                            odoAngleCorrection  = clamp(odoAngleCorrection,
                                    -ODO_CORRECTION_MAX_DEG, ODO_CORRECTION_MAX_DEG);
                        }
                    }

                    // Keep commandedAngle in sync with wherever FINE_ALIGN has
                    // moved the servo so SEEKING resumes from the right place.
                    commandedAngle = normalizeDegrees(servoToAngle(currentServoPos));
                }
                // Tag absent: FF correction already applied, position held.

                setServos(currentServoPos);
                firstRun       = false;
                lastUpdateTime = currentTime;
                return;
            }

            // ════════════════════════════════════════════════════════════
            // STEP 8B — SEEKING: odometry bearing PID (residual correction)
            //
            // Feedforward already compensated for the heading change this
            // loop. The PID handles the residual: heading lag, odometry
            // drift, and any initial bearing error at startup.
            //
            // WRAPPING-SAFE INTEGRATION:
            //   odoError is computed in angle space with normalizeDegrees()
            //   (always ±180°, always shortest path). The PID output is
            //   converted to a servo delta and ADDED to currentServoPos —
            //   a continuous value that is never renormalized. This avoids
            //   the ±180° snap that caused TurretOdoAi to slam to its stop
            //   whenever commandedAngle crossed the wrap boundary.
            //
            //   Conversion: pidDegPerSec × dt = Δangle (deg)
            //               Δpos = −Δangle / 350  (inverted mapping)
            // ════════════════════════════════════════════════════════════

            // Recompute odoError relative to commandedAngle (FF moved currentServoPos;
            // commandedAngle tracks where we've told the turret to go in angle space).
            // Using commandedAngle rather than servoToAngle(currentServoPos) keeps the
            // PID smooth even when FF produces small floating-point rounding differences.
            turretAngleDeg = servoToAngle(currentServoPos);
            double odoErrorPost = targetAngleDeg - commandedAngle;
            if (odoErrorPost >  180) odoErrorPost -= 360;
            if (odoErrorPost < -180) odoErrorPost += 360;

            if (Math.abs(odoErrorPost) < TOLERANCE) {
                seekLastError  = odoErrorPost;
                setServos(currentServoPos);
                firstRun       = false;
                lastUpdateTime = currentTime;
                return;
            }

            double P_out  = kP_seek * odoErrorPost;
            seekIntegral += odoErrorPost * dt;
            seekIntegral  = clamp(seekIntegral, -100.0, 100.0);
            double I_out  = kI_seek * seekIntegral;
            double D_out  = kD_seek * ((odoErrorPost - seekLastError) / dt);
            double pidDegPerSec = clamp(P_out + I_out + D_out, -MAX_VELOCITY, MAX_VELOCITY);

            // Advance commandedAngle in angle space (normalize keeps it in ±180°)
            commandedAngle = normalizeDegrees(commandedAngle + pidDegPerSec * dt);

            // Convert to servo delta (inverted mapping: angle↑ → pos↓) and accumulate
            double seekServoDelta = -(pidDegPerSec * dt) / DEG_PER_SERVO_UNIT;
            currentServoPos = clamp(currentServoPos + seekServoDelta, SERVO_MIN, SERVO_MAX);

            setServos(currentServoPos);

            seekLastError  = odoErrorPost;
            firstRun       = false;
            lastUpdateTime = currentTime;

        } catch (Exception e) {
            // Never crash the OpMode loop
        }
    }

    // ═════════════════════════════════════════════════════════════════════
    // POSE CORRECTION — opportunistic 3D Limelight
    // ═════════════════════════════════════════════════════════════════════
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
            // Opportunistic — ignore silently
        }
    }

    // ═════════════════════════════════════════════════════════════════════
    // RELOCALIZATION — manual trigger
    // ═════════════════════════════════════════════════════════════════════
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

            double currentHeadingRad = PedroComponent.follower().getPose().getHeading();
            PedroComponent.follower().setPose(new Pose(llX, llY, currentHeadingRad));

            // Force SEEKING so the odometry PID immediately drives the turret
            // to the bearing implied by the corrected pose. Without this the
            // turret would hold its current position even though the pose just
            // changed, and FINE_ALIGN would re-engage on stale odoError values.
            state         = TurretState.SEEKING;
            seekIntegral  = 0;
            seekLastError = 0;
            consecutiveDetections = 0;
            consecutiveMisses     = 0;
            return true;
        } catch (Exception e) {
            return false;
        }
    }

    // ═════════════════════════════════════════════════════════════════════
    // HELPERS
    // ═════════════════════════════════════════════════════════════════════

    /** Commands both gearbox servos to the same position. */
    private void setServos(double position) {
        turretServo1.setPosition(position);
        turretServo2.setPosition(position);
    }

    public void turnRight() { ManualAngleAdjust += 2; }
    public void turnLeft()  { ManualAngleAdjust -= 2; }

    /** Zeroes the accumulated tx-based bearing correction (call at match start if needed). */
    public void resetOdoCorrection() { odoAngleCorrection = 0.0; }

    private double servoToAngle(double servoPos) {
        return normalizeDegrees(DEG_PER_SERVO_UNIT * (1.0 - servoPos) - 180.0);
    }

    /** Used only for init/diagnostics — never in the control loop. */
    private double angleToServo(double angleDeg) {
        angleDeg = normalizeDegrees(angleDeg);
        return clamp(1.0 - ((angleDeg + 180.0) / DEG_PER_SERVO_UNIT), SERVO_MIN, SERVO_MAX);
    }

    public double normalizeDegrees(double angle) {
        while (angle >  180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    // ═════════════════════════════════════════════════════════════════════
    // GETTERS
    // ═════════════════════════════════════════════════════════════════════
    public double      getX()                        { return x; }
    public double      getY()                        { return y; }
    public double      getHeading()                  { return heading; }
    public double      getHeadingRate()              { return headingRate; }
    public double      getTargetAngleDeg()           { return targetAngleDeg; }
    public double      getTurretAngleDeg()           { return turretAngleDeg; }
    public double      getDistanceToTarget()         { return distanceToTarget; }
    public double      getLastOdoError()             { return lastOdoError; }
    public double      getLastSeekError()            { return seekLastError; }
    public double      getSeekIntegral()             { return seekIntegral; }
    public int         getSkippedLoops()             { return skippedLoops; }
    public boolean     isManualMode()                { return manualMode; }
    public double      getCurrentServoPos()          { return currentServoPos; }
    public boolean     isFineAligned()               { return state == TurretState.FINE_ALIGN; }
    public boolean     isLimelightLocked()           { return state == TurretState.FINE_ALIGN; }
    public int         getLimelightCorrectionCount() { return limelightCorrectionCount; }
    public double      getLastPoseCorrectionMag()    { return lastPoseCorrectionMag; }
    public TurretState getState()                    { return state; }
    public int         getConsecutiveDetections()    { return consecutiveDetections; }
    public int         getConsecutiveMisses()        { return consecutiveMisses; }
    public double      getLastTx()                   { return lastTx; }
    public double      getLastFineServoDelta()       { return lastFineServoDelta; }
    public double      getLastFFServoDelta()         { return lastFFServoDelta; }
    public double      getAllianceTxOffset()          { return allianceTxOffset; }
    public double      getOdoAngleCorrection()       { return odoAngleCorrection; }
    public double      getCommandedAngle()           { return commandedAngle; }
}