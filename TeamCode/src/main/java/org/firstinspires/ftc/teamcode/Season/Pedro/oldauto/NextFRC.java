package org.firstinspires.ftc.teamcode.Season.Pedro.oldauto;

import org.firstinspires.ftc.teamcode.Season.Pedro.Constants;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual1Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual1Subsystem.Midtake;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual1Subsystem.Outtake;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
@Disabled
@Autonomous(name = "Movement Auto (with Intake)", group = "Examples")
public class NextFRC extends NextFTCOpMode {

    public NextFRC() {
        addComponents(
                new SubsystemComponent(Outtake.INSTANCE, Intake.INSTANCE, Midtake.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(123.13, 122.08, Math.toRadians(220));
    private final Pose scorePose = new Pose(95.37, 94.74, Math.toRadians(230));

    private final Pose prePickup1 = new Pose(82.226, 81.809, Math.toRadians(0));
    private final Pose prePickup2 = new Pose(80.765, 59.270, Math.toRadians(0));
    private final Pose prePickup3 = new Pose(85.565, 34.017, Math.toRadians(0));

    private final Pose pickup1Pose = new Pose(118, 84, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(118, 60, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(118, 35, Math.toRadians(0));

    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;
    private PathChain grabPrePickup1, grabPrePickup2, grabPrePickup3;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        grabPrePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, prePickup1))
                .setLinearHeadingInterpolation(scorePose.getHeading(), prePickup1.getHeading())
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(prePickup1, pickup1Pose))
                .setLinearHeadingInterpolation(prePickup1.getHeading(), pickup1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPrePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, prePickup2))
                .setLinearHeadingInterpolation(scorePose.getHeading(), prePickup2.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(prePickup2, pickup2Pose))
                .setLinearHeadingInterpolation(prePickup2.getHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPrePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, prePickup3))
                .setLinearHeadingInterpolation(scorePose.getHeading(), prePickup3.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(prePickup3, pickup3Pose))
                .setLinearHeadingInterpolation(prePickup3.getHeading(), pickup3Pose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(grabPrePickup1, true);
                    Intake.INSTANCE.activeintake.setPower(1); // 🔹 turn ON intake
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup1, true);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    Intake.INSTANCE.activeintake.setPower(0); // 🔹 turn OFF intake before scoring
                    follower.followPath(scorePickup1, true);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(grabPrePickup2, true);
                    Intake.INSTANCE.activeintake.setPower(1); // 🔹 turn ON intake again
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup2, true);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    Intake.INSTANCE.activeintake.setPower(0); // 🔹 turn OFF intake before scoring
                    follower.followPath(scorePickup2, true);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(grabPrePickup3, true);
                    Intake.INSTANCE.activeintake.setPower(1); // 🔹 turn ON intake
                    setPathState(8);
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup3, true);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    Intake.INSTANCE.activeintake.setPower(0); // 🔹 final OFF before last score
                    follower.followPath(scorePickup3, true);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    setPathState(-1); // Stop auto
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void onUpdate() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void onInit() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override public void onWaitForStart() { }

    @Override public void onStartButtonPressed() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override public void onStop() {
        Intake.INSTANCE.activeintake.setPower(0); // safety stop intake
    }
}