package org.firstinspires.ftc.teamcode.Season.Auto.RegionalsAuto;

import org.firstinspires.ftc.teamcode.Season.Auto.Constants;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.CompliantIntake;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.ManualTurret;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.NewHood;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.RobotContext;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.ShooterPID;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.Transfer;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.ftc.components.LoopTimeComponent;

@Autonomous(name = "RedFarClaude", group = "Examples")
public class RedFarClaude extends NextFTCOpMode {

    public RedFarClaude() {
        addComponents(
                new SubsystemComponent(NewHood.INSTANCE, CompliantIntake.INSTANCE, Transfer.INSTANCE, ManualTurret.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE, new LoopTimeComponent()
        );
    }

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    // Shoot timing (seconds) — replaces sleep() calls
    private static final double FLYWHEEL_TIMEOUT  = 1.0;
    private static final double BELT_RUN_DURATION = 0.6;  // was sleep(600) in original

    // Preload needs slightly higher velocity (was 1450 in original scheduleOuttake)
    // dist from (87,17) to (130,130) ≈ 120.9 → equation gives ≈ 1554
    private static final double SHOOT_VELO         = 1554;
    private static final double SHOOT_VELO_PRELOAD = 1554; // same distance, adjust if needed

    // ── Poses ─────────────────────────────────────────────────────────────
    private final Pose startPose = new Pose(87.707, 8.328, Math.toRadians(0));
    private final Pose scorePose = new Pose(87.031, 17.521, Math.toRadians(0));

    // ── Paths ─────────────────────────────────────────────────────────────
    private Path scorePreload;
    private PathChain grabPrePickup1, scorePickup1, grabPrePickup2, scorePickup2, grabPickup3, scorePickup3;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        grabPrePickup1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(87.031, 17.521),
                                new Pose(85.540, 36.086),
                                new Pose(131.133, 37.447)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(131.133, 37.447),
                                new Pose(86.925, 17.383)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        grabPrePickup2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(86.925, 17.383),
                                new Pose(135.207, 8.640)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(135.207, 8.640),
                                new Pose(87.161, 17.288)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(87.161, 17.288),
                                new Pose(133.751, 20.479)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                new Pose(133.751, 20.479),
                                new Pose(133.929, 15.597)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(133.929, 15.597),
                                new Pose(87.191, 17.537)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    // ── State machine ─────────────────────────────────────────────────────
    //
    // Shoot sequences use two shared states (100 & 101):
    //   100 — wait for atTargetVelocity() or FLYWHEEL_TIMEOUT, then start belt
    //   101 — run belt for BELT_RUN_DURATION, then stop and go to nextStateAfterShoot
    //
    // Set nextStateAfterShoot before jumping to state 100.
    //
    private int nextStateAfterShoot = -1;

    public void autonomousPathUpdate() {
        switch (pathState) {

            // ── Preload ───────────────────────────────────────────────────
            case 0:
                NewHood.INSTANCE.midclose();
                ManualTurret.INSTANCE.setPosition(0.93);
                ShooterPID.INSTANCE.setTargetVelocity(SHOOT_VELO_PRELOAD);
                follower.followPath(scorePreload, true);
                setPathState(1);
                break;

            case 1: // Wait for drive, then shoot
                if (!follower.isBusy()) {
                    nextStateAfterShoot = 2;
                    setPathState(100);
                }
                break;

            case 2: // Post-preload: intake + drive to pickup1
                CompliantIntake.INSTANCE.on();
                Transfer.INSTANCE.repel();
                follower.followPath(grabPrePickup1);
                setPathState(3);
                break;

            case 3: // Wait for pickup1, then return to score
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup1);
                    ShooterPID.INSTANCE.setTargetVelocity(SHOOT_VELO);
                    setPathState(4);
                }
                break;

            case 4: // Wait to arrive at score, then shoot
                if (!follower.isBusy()) {
                    nextStateAfterShoot = 5;
                    setPathState(100);
                }
                break;

            // ── Pickup 2 ──────────────────────────────────────────────────
            case 5:
                CompliantIntake.INSTANCE.on();
                Transfer.INSTANCE.repel();
                follower.followPath(grabPrePickup2);
                setPathState(6);
                break;

            case 6:
                if (!follower.isBusy()) {
                    CompliantIntake.INSTANCE.on();
                    Transfer.INSTANCE.repel();
                    ShooterPID.INSTANCE.setTargetVelocity(SHOOT_VELO);
                    follower.followPath(scorePickup2);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    nextStateAfterShoot = 8;
                    setPathState(100);
                }
                break;

            // ── Pickup 3 ──────────────────────────────────────────────────
            case 8:
                CompliantIntake.INSTANCE.on();
                Transfer.INSTANCE.repel();
                follower.followPath(grabPickup3, true);
                ShooterPID.INSTANCE.setTargetVelocity(SHOOT_VELO);
                setPathState(9);
                break;

            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup3);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    nextStateAfterShoot = -1;
                    setPathState(100);
                }
                break;

            // ── Shared shoot sequence ─────────────────────────────────────
            case 100: // Wait for flywheel, then start belt
                if (ShooterPID.INSTANCE.atTargetVelocity() || pathTimer.getElapsedTimeSeconds() > FLYWHEEL_TIMEOUT) {
                    CompliantIntake.INSTANCE.on();
                    Transfer.INSTANCE.on();
                    setPathState(101);
                }
                break;

            case 101: // Run belt for fixed duration, then stop + continue
                if (pathTimer.getElapsedTimeSeconds() > BELT_RUN_DURATION) {
                    CompliantIntake.INSTANCE.off();
                    Transfer.INSTANCE.off();
                    ShooterPID.INSTANCE.stop();
                    setPathState(nextStateAfterShoot);
                }
                break;
        }
    }

    private void savePose() {
        RobotContext.lastPose = follower.getPose();
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void onUpdate() {
        ShooterPID.INSTANCE.update();
        follower.update();
        autonomousPathUpdate();
        telemetry.addData("Velocity",      ShooterPID.INSTANCE.getActualVelocity());
        telemetry.addData("At Target",     ShooterPID.INSTANCE.atTargetVelocity());
        telemetry.addData("Path State",    pathState);
        telemetry.addData("X",             follower.getPose().getX());
        telemetry.addData("Y",             follower.getPose().getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void onInit() {
        ShooterPID.init(hardwareMap);
        pathTimer   = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        NewHood.INSTANCE.init(hardwareMap);
        ManualTurret.INSTANCE.init(hardwareMap);
        NewHood.INSTANCE.setAlliance("red");
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override public void onWaitForStart() {}

    @Override public void onStartButtonPressed() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override public void onStop() {
        savePose();
        ShooterPID.INSTANCE.stop();
    }
}