package org.firstinspires.ftc.teamcode.Season.Pedro.qual2auto;

import org.firstinspires.ftc.teamcode.Season.Pedro.Constants;
import org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems.RedExperimentalDistanceLExtractor;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.AutoOuttake;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.CompliantIntake;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.Hood;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.RedLL;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.RobotContext;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.Transfer;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.TurretPID;
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

@Autonomous(name = "RedFar", group = "Examples")
public class RedFar extends NextFTCOpMode {
    VoltageGet volt = new VoltageGet();
    public RedFar() {
        addComponents(
                new SubsystemComponent(AutoOuttake.INSTANCE, Hood.INSTANCE, CompliantIntake.INSTANCE,Transfer.INSTANCE, Turret.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }
    private VoltageGet voltageGet;

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(87.3137137494973, 8.174520756065958, Math.toRadians(0));
    //    private final Pose scorePose = new Pose(90, 90, Math.toRadians(215));
    private final Pose scorePose = new Pose(132.8115107913669, 12.846043165467632, Math.toRadians(0));

    private final Pose preprePickup1 = new Pose(12, 12, Math.toRadians(0));
    private final Pose prePickup1 = new Pose(17.581, 10.0470, Math.toRadians(0));
    private final Pose prePickup2 = new Pose(126.207, 59.379, Math.toRadians(0)); //55
    private final Pose prePickup3 = new Pose(46.5, 84, Math.toRadians(0));
    //    private final Pose dropoff2 = new Pose(100, 54, Math.toRadians(180)); //55
    private final Pose pickup1Pose = new Pose(126.41379310344828, 87, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(9, 52, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(12, 84, Math.toRadians(0));

    private Path scorePreload;
    private RedExperimentalDistanceLExtractor limelight;
    private RedLL turretAlignment;


    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;
    private PathChain grabPrePickup1, grabPrePickup2, grabPrePickup3, gatepickup, curvescore,leave,grabPickup4,scorePickup4;

    private PathChain dropofftwo;

    public void buildPaths() {


        grabPrePickup1 = follower.pathBuilder()
//                .addPath(
//                        new BezierCurve(
//                                new Pose(96.000, 96.000),
//                                new Pose(67.241, 78.414),
//                                new Pose(117, 83.79310344827586)
//                        )
//                )
                .addPath(
                        new BezierCurve(
                                new Pose(87.314, 9),
                                new Pose(89.715, 9),
                                new Pose(130, 9)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))


                .addPath(
                        new BezierLine(
                                new Pose(130, 9 ),

                                new Pose(130, 12.846)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();




        grabPrePickup2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(83.706, 11.810),
                                new Pose(88.886, 37.502),
                                new Pose(125, 36.259)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();
        scorePickup1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(130, 12.846),

                                new Pose(83.706, 11.810)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();
        scorePickup2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(125, 36.259),

                                new Pose(83.292, 11.810)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();
//        grabPickup2 = follower.pathBuilder()
//
//                .build();
////        dropofftwo = follower.pathBuilder()
////                .addPath(new BezierLine(pickup2Pose,dropoff2))
////                .setLinearHeadingInterpolation(pickup2Pose.getHeading(),dropoff2.getHeading())
////                .build();
//
        leave = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(83.292, 12.017),

                                new Pose(98.632, 23.219)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(87.314, 8.175),
                                new Pose(89.715, 8.288),
                                new Pose(125, 7.666)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))


                .addPath(
                        new BezierLine(
                                new Pose(125, 7.666),

                                new Pose(125, 12.846)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(125, 12.846),

                                new Pose(83.292, 12.017)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        grabPickup4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(83.292, 12.017),
                                new Pose(89.715, 8.288),
                                new Pose(125, 7.666)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))


                .addPath(
                        new BezierLine(
                                new Pose(125, 7.666),

                                new Pose(125, 12.846)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        scorePickup4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(125, 12.846),

                                new Pose(83.292, 12.017)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
               scheduleOuttake();
                Hood.INSTANCE.midclose();
                turretAlignment.farAuto();
                setPathState(1);

                break;

            case 1:
                if (!follower.isBusy()) {

                    letmotorspeed();
                    // SHOOT after preload

                    Intake();
                    follower.followPath(grabPrePickup1);

                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    Transfer.INSTANCE.off();
//                    CompliantIntake.INSTANCE.off();
//                    TurretPID.INSTANCE.setFarShooterSpeed();
                    scheduleOuttake();
                    follower.followPath(scorePickup1, true);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    turretAlignment.align();
                    letmotorspeed();
                    Intake();
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    setPathState(6);
                }
                break;
//
            case 6:
                if (!follower.isBusy()) {
                    turretAlignment.align();
                    letmotorspeed();
                    Intake();
                    follower.followPath(grabPickup4);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    secondshotforyouuuuu();
                    follower.followPath(scorePickup4);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    letmotorspeed();
                    follower.followPath(leave);
                  //intake again
                    setPathState(-1);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    //shoot the 15th
                    setPathState(10);
                }
                break;
//
            case 10:
                if (!follower.isBusy()) {
                    shootThreeBalls();
                    follower.followPath(leave);
                    savePose();
                    setPathState(-1);

                }
                break;
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
    private void savePose() { // runs when auto finishes
        RobotContext.lastPose = follower.getPose();
    }
    private void scheduleOuttake() {
        AutoOuttake.INSTANCE.setShooterSpeed(1550).schedule();
    }
    private void secondshotforyouuuuu() {
        AutoOuttake.INSTANCE.setShooterSpeed(1550).schedule();
    }
    private void Intake() {
        CompliantIntake.INSTANCE.on();
        Transfer.INSTANCE.advance();
    }
    private void setShooterFromOdometry() {
        Pose pose = follower.getPose();
        double distance = Math.hypot(
                96 - pose.getX(),  // TARGET_X = 96 (your scorePose X)
                96 - pose.getY()   // TARGET_Y = 96 (your scorePose Y)
        );
        TurretPID.INSTANCE.newshooterdistance(distance).schedule();
    }

    private void letmotorspeed() {
        sleep(1300);
        CompliantIntake.INSTANCE.on();
        Transfer.INSTANCE.on();
        sleep(700);
        AutoOuttake.INSTANCE.setShooterSpeed(1520);
        CompliantIntake.INSTANCE.off();
        Transfer.INSTANCE.off();
        sleep(700);
        CompliantIntake.INSTANCE.on();
        Transfer.INSTANCE.on();
        AutoOuttake.INSTANCE.setShooterSpeed(1515);
        sleep(700);
        CompliantIntake.INSTANCE.off();
        Transfer.INSTANCE.off();
    }
    private void shootThreeBalls() {
        turretAlignment.align();
        CompliantIntake.INSTANCE.on();
        Transfer.INSTANCE.on();
        sleep(1300);
        CompliantIntake.INSTANCE.off();
        Transfer.INSTANCE.off();
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void onUpdate() {
        if(pathState==0){
            turretAlignment.farAuto();
        }
        if(pathState<2){
            turretAlignment.farAuto();
        }
        if (pathState==3){
            turretAlignment.farAuto();
        }
        if (pathState == 4){
            turretAlignment.farAuto();
        }
        if (pathState == 5){
            turretAlignment.farAuto();
        }
        if (pathState == 6){
            turretAlignment.farAuto();
        }
        if (pathState == 7){
            turretAlignment.farAuto();
        }
        if (pathState == 8){
            turretAlignment.farAuto();
        }
//        if(pathState==0){
//            turretAlignment.farAlign();
//        }
//        if(pathState<2){
//            turretAlignment.farAlign();
//        }
//        if (pathState==3){
//            turretAlignment.farAlign();
//        }
//        if (pathState == 4){
//            turretAlignment.farAlign();
//        }
//        if (pathState == 5){
//            turretAlignment.farAlign();
//        }
//        if (pathState == 6){
//            turretAlignment.farAlign();
//        }
//        if (pathState == 7){
//            turretAlignment.farAlign();
//        }
//        if (pathState == 8){
//            turretAlignment.farAlign();
//        }
        follower.update();
        turretAlignment.align();
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
        limelight = new RedExperimentalDistanceLExtractor(hardwareMap);
        turretAlignment = new RedLL(hardwareMap, limelight,    voltageGet);
        limelight.startReading();
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
        AutoOuttake.INSTANCE.resetShooter().schedule();

    }

}