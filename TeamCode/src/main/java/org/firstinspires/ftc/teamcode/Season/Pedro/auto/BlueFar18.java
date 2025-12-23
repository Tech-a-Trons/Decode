package org.firstinspires.ftc.teamcode.Season.Pedro.auto;

import static org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual1Subsystem.Outtake.outtake;

import org.firstinspires.ftc.teamcode.Season.Pedro.Constants;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual1Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual1Subsystem.Midtake;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual1Subsystem.Outtake;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.CompliantIntake;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.Hood;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.TurretPID;
import org.firstinspires.ftc.teamcode.Season.Subsystems.Sensors.VoltageGet;

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

@Autonomous(name = "BlueAuto18?", group = "Examples")
public class BlueFar18 extends NextFTCOpMode {
    VoltageGet volt = new VoltageGet();
    public BlueFar18() {
        addComponents(
                new SubsystemComponent(TurretPID.INSTANCE, Hood.INSTANCE, CompliantIntake.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }


    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(56, 8.000, Math.toRadians(90));
    //    private final Pose scorePose = new Pose(90, 90, Math.toRadians(215));
    private final Pose scorePose = new Pose(54.5, 17.5, Math.toRadians(180));

    private final Pose preprePickup1 = new Pose(12, 12, Math.toRadians(180));
    private final Pose prePickup1 = new Pose(17.581, 10.0470, Math.toRadians(180));
    private final Pose prePickup2 = new Pose(48, 52, Math.toRadians(180)); //55
    private final Pose prePickup3 = new Pose(46.5, 84, Math.toRadians(180));
    //    private final Pose dropoff2 = new Pose(100, 54, Math.toRadians(180)); //55
    private final Pose pickup1Pose = new Pose(12, 11, Math.toRadians(180));
    private final Pose pickup2Pose = new Pose(9, 52, Math.toRadians(180));
    private final Pose pickup3Pose = new Pose(12, 84, Math.toRadians(180));

    private Path scorePreload;


    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;
    private PathChain grabPrePickup1, grabPrePickup2, grabPrePickup3;

    private PathChain dropofftwo;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        grabPrePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, preprePickup1))
                .setLinearHeadingInterpolation(scorePose.getHeading(), preprePickup1.getHeading())
                .addPath(new BezierLine(preprePickup1,prePickup1))
                .setLinearHeadingInterpolation(preprePickup1.getHeading(), prePickup1.getHeading())
                .addPath(new BezierLine(prePickup1, pickup1Pose))
                .setLinearHeadingInterpolation(prePickup1.getHeading(), pickup1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();
//        grabPrePickup2 = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose, prePickup2))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), prePickup2.getHeading())
//                .build();
//
//        grabPickup2 = follower.pathBuilder()
//                .addPath(new BezierLine(prePickup2, pickup2Pose))
//                .setLinearHeadingInterpolation(prePickup2.getHeading(), pickup2Pose.getHeading())
//                .build();
////        dropofftwo = follower.pathBuilder()
////                .addPath(new BezierLine(pickup2Pose,dropoff2))
////                .setLinearHeadingInterpolation(pickup2Pose.getHeading(),dropoff2.getHeading())
////                .build();
//        scorePickup2 = follower.pathBuilder()
//                .addPath(new BezierLine(pickup2Pose, scorePose))
//                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
//                .build();
//
//        grabPrePickup3 = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose, prePickup3))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), prePickup3.getHeading())
//                .build();
//
//        grabPickup3 = follower.pathBuilder()
//                .addPath(new BezierLine(prePickup3, pickup3Pose))
//                .setLinearHeadingInterpolation(prePickup3.getHeading(), pickup3Pose.getHeading())
//                .build();
//
//        scorePickup3 = follower.pathBuilder()
//                .addPath(new BezierLine(pickup3Pose, scorePose))
//                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
//                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload, true);
                setPathState(1);

                break;

            case 1:
                if (!follower.isBusy()) {
                    // SHOOT after preload
                    shootThreeBalls();
                    follower.followPath(grabPrePickup1);

                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
//                    CompliantIntake.INSTANCE.off();
//                    TurretPID.INSTANCE.setFarShooterSpeed();
                    follower.followPath(scorePickup1, true);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    shootThreeBalls();
                    setPathState(4);
                }
                break;

//            case 4:
//                if (!follower.isBusy()) {
//                    shootThreeBalls();
////                    follower.followPath(grabPrePickup2, true);
////                    Intake.INSTANCE.activeintake.setPower(1);
////                    Midtake.INSTANCE.newtake.setPower(0.3);
//                    setPathState(5);
//                }
//                break;
//
//            case 5:
//                if (!follower.isBusy()) {
//                    follower.setMaxPower(0.8);
//                    follower.followPath(grabPickup2, true);
//                    follower.setMaxPower(1);
//                    setPathState(6);
//                }
//                break;
//
//            case 6:
//                if (!follower.isBusy()) {
//                    Midtake.INSTANCE.newtake.setPower(0);
//                    Intake.INSTANCE.activeintake.setPower(0);
////                    Outtake.INSTANCE.outtake.setPower(0.1);
//                    follower.followPath(scorePickup2, true);
//                    setPathState(7);
//                }
//                break;
//
//            case 7:
//                if (!follower.isBusy()) {
//                    thirdshootThreeBalls();
//                    follower.followPath(grabPrePickup3, true);
//                    Intake.INSTANCE.activeintake.setPower(1);
//                    Midtake.INSTANCE.newtake.setPower(0.3);
//                    setPathState(8);
//                }
//                break;
//
//            case 8:
//                if (!follower.isBusy()) {
//                    follower.followPath(grabPickup3, true);
//                    setPathState(9);
//                }
//                break;
//
//            case 9:
//                if (!follower.isBusy()) {
//                    Midtake.INSTANCE.newtake.setPower(0);
//                    Intake.INSTANCE.activeintake.setPower(0);
//                    follower.followPath(scorePickup3, true);
////                    shootThreeBalls();
//                    setPathState(10);
////                }
//                break;
//
//            case 10:
//                if (!follower.isBusy()) {
//                    secondshootThreeBalls();
//                    setPathState(-1);
//                }
//                break;
        }
    }

    private void secondshootThreeBalls() {

    }
    private void Intake() {
//        CompliantIntake.INSTANCE.on();
//        Transfer.INSTANCE.repel();
    }
    private void shootThreeBalls() {
//    TurretPID.INSTANCE.setFarShooterSpeed();
//    Hood.INSTANCE.midclose();
//    CompliantIntake.INSTANCE.on();
//    Transfer.INSTANCE.on();
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
        volt.init(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override public void onWaitForStart() {}

    @Override public void onStartButtonPressed() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override public void onStop() {
        Intake.INSTANCE.activeintake.setPower(0);
        outtake.setPower(0);
        Midtake.newtake.setPower(0);
    }

}