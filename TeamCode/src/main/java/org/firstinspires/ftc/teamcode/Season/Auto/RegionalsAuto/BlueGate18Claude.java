package org.firstinspires.ftc.teamcode.Season.Auto.RegionalsAuto;


import org.firstinspires.ftc.teamcode.Season.Auto.Constants;
import org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems.RedExperimentalDistanceLExtractor;

import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.CompliantIntake;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.ManualTurret;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.NewHood;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.RobotContext;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.ShooterPID;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.Transfer;
import org.firstinspires.ftc.teamcode.Season.Subsystems.Sensors.VoltageGet;

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

@Autonomous(name = "RedGate18Claude", group = "Examples")
public class BlueGate18Claude extends NextFTCOpMode {

    public BlueGate18Claude() {
        addComponents(
                new SubsystemComponent(NewHood.INSTANCE, CompliantIntake.INSTANCE, Transfer.INSTANCE, ManualTurret.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE, new LoopTimeComponent()
        );
    }

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    // Shoot timing (seconds) — replaces sleep() calls in shootThreeBalls
    private static final double FLYWHEEL_TIMEOUT  = 0.7;
    private static final double BELT_RUN_DURATION = 0.5;

    // ── Poses ─────────────────────────────────────────────────────────────
    private final Pose startPose = new Pose(123.22766570605188, 123.15850144092221, Math.toRadians(35)).mirror();
    private final Pose scorePose = new Pose(85, 85, Math.toRadians(0)).mirror();

    // ── Paths ─────────────────────────────────────────────────────────────
    private Path scorePreload;
    private PathChain grabPrePickup1, grabPrePickup2, scorePickup2, grabPickup3, scorePickup3, leave;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        grabPrePickup1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(85.000, 85.000).mirror(),
                                new Pose(95.669, 53.833).mirror(),
                                new Pose(170, 59.524).mirror()
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .addPath(
                        new BezierCurve(
                                new Pose(170, 59.524).mirror(),
                                new Pose(94.007, 60.817).mirror(),
                                new Pose(85.000, 85.000).mirror()
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        grabPrePickup2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(85.000, 85.000).mirror(),
                                new Pose(93.635, 58.775).mirror(),
                                new Pose(132, 61.300).mirror()
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(25+180))
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(129, 61.300).mirror(),
                                new Pose(86.203, 68.473).mirror(),
                                new Pose(85.000, 85.000).mirror()
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(25+180), Math.toRadians(180))
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(85.000, 85.000).mirror(),
                                new Pose(130, 84.421).mirror()
                        )
                ).setTangentHeadingInterpolation()
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(130, 84.421).mirror(),
                                new Pose(85.000, 85.000).mirror()
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        leave = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(85.000, 85.000).mirror(),
                                new Pose(103.187, 75.752).mirror()
                        )
                ).setTangentHeadingInterpolation()
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
                ManualTurret.INSTANCE.setPosition(0);
                ShooterPID.INSTANCE.setTargetVelocity(1231);
                follower.followPath(scorePreload, true);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    nextStateAfterShoot = 2;
                    setPathState(100);
                }
                break;

            case 2: // Post-preload shoot: start intake + drive to grabPrePickup1
                CompliantIntake.INSTANCE.on();
                Transfer.INSTANCE.repel();
                ShooterPID.INSTANCE.setTargetVelocity(1231);
                follower.followPath(grabPrePickup1);
                setPathState(3);
                break;

            case 3:
                if (!follower.isBusy()) {
                    Transfer.INSTANCE.off();
                    nextStateAfterShoot = 4;
                    setPathState(100);
                }
                break;

            // ── Gate cycle 1 ──────────────────────────────────────────────
            case 4:
                CompliantIntake.INSTANCE.on();
                Transfer.INSTANCE.repel();
                follower.followPath(grabPrePickup2);
                setPathState(5);
                break;

            case 5:
                if (!follower.isBusy()) {
                    GateIntake();
                    ShooterPID.INSTANCE.setTargetVelocity(1231);
                    follower.followPath(scorePickup2);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    nextStateAfterShoot = 7;
                    setPathState(100);
                }
                break;

            // ── Gate cycle 2 ──────────────────────────────────────────────
            case 7:
                CompliantIntake.INSTANCE.on();
                Transfer.INSTANCE.repel();
                follower.followPath(grabPrePickup2);
                setPathState(8);
                break;

            case 8:
                if (!follower.isBusy()) {
                    GateIntake();
                    ShooterPID.INSTANCE.setTargetVelocity(1231);
                    follower.followPath(scorePickup2);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    nextStateAfterShoot = 10;
                    setPathState(100);
                }
                break;

            // ── Gate cycle 3 ──────────────────────────────────────────────
            case 10:
                CompliantIntake.INSTANCE.on();
                Transfer.INSTANCE.repel();
                follower.followPath(grabPrePickup2);
                setPathState(11);
                break;

            case 11:
                if (!follower.isBusy()) {
                    GateIntake();
                    ShooterPID.INSTANCE.setTargetVelocity(1231);
                    follower.followPath(scorePickup2);
                    setPathState(12);
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    nextStateAfterShoot = 13;
                    setPathState(100);
                }
                break;

            // ── Field pickup ──────────────────────────────────────────────
            case 13:
                ShooterPID.INSTANCE.setTargetVelocity(1231);
                follower.followPath(grabPickup3, true);
                setPathState(14);
                break;

            case 14:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup3);
                    setPathState(15);
                }
                break;

            case 15:
                if (!follower.isBusy()) {
                    nextStateAfterShoot = 16;
                    setPathState(100);
                }
                break;

            case 16:
                follower.followPath(leave);
                setPathState(-1);
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

    // GateIntake sleep is intentional — shooter is off during this window
    private void GateIntake() {
        CompliantIntake.INSTANCE.on();
        Transfer.INSTANCE.repel();
        sleep(1200);
        Transfer.INSTANCE.off();
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