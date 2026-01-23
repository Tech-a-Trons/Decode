//package org.firstinspires.ftc.teamcode.Season.Pedro.auto;
//
//import static org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual1Subsystem.Outtake.outtake;
//
//import org.firstinspires.ftc.teamcode.Season.Pedro.Constants;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems.RedExperimentalDistanceLExtractor;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual1Subsystem.Intake;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual1Subsystem.Midtake;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual1Subsystem.Outtake;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.CompliantIntake;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.Hood;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.SimpleLL;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.Transfer;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.Turret;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.TurretPID;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.Sensors.VoltageGet;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.Path;
//import com.pedropathing.paths.PathChain;
//import com.pedropathing.util.Timer;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//
//import dev.nextftc.core.components.BindingsComponent;
//import dev.nextftc.core.components.SubsystemComponent;
//import dev.nextftc.ftc.NextFTCOpMode;
//import dev.nextftc.ftc.components.BulkReadComponent;
//
//@Autonomous(name = "RedGateClose", group = "Examples")
//public class RedGateClose extends NextFTCOpMode {
//    VoltageGet volt = new VoltageGet();
//    public RedGateClose() {
//        addComponents(
//                new SubsystemComponent(TurretPID.INSTANCE, Hood.INSTANCE, CompliantIntake.INSTANCE,Transfer.INSTANCE, Turret.INSTANCE),
//                BulkReadComponent.INSTANCE,
//                BindingsComponent.INSTANCE
//        );
//    }
//
//
//    private Follower follower;
//    private Timer pathTimer, opmodeTimer;
//    private int pathState;
//
//    private final Pose startPose = new Pose(122.6, 122.5, Math.toRadians(310));
//    //    private final Pose scorePose = new Pose(90, 90, Math.toRadians(215));
//    private final Pose scorePose = new Pose(96, 96, Math.toRadians(0));
//
//    private final Pose preprePickup1 = new Pose(12, 12, Math.toRadians(180));
//    private final Pose prePickup1 = new Pose(17.581, 10.0470, Math.toRadians(180));
//    private final Pose prePickup2 = new Pose(126.207, 59.379, Math.toRadians(180)); //55
//    private final Pose prePickup3 = new Pose(46.5, 84, Math.toRadians(180));
//    //    private final Pose dropoff2 = new Pose(100, 54, Math.toRadians(180)); //55
//    private final Pose pickup1Pose = new Pose(126.41379310344828, 87, Math.toRadians(0));
//    private final Pose pickup2Pose = new Pose(9, 52, Math.toRadians(180));
//    private final Pose pickup3Pose = new Pose(12, 84, Math.toRadians(180));
//
//    private Path scorePreload;
//    private RedExperimentalDistanceLExtractor limelight;
//    private SimpleLL turretAlignment;
//
//
//    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;
//    private PathChain grabPrePickup1, grabPrePickup2, grabPrePickup3, gatepickup, curvescore;
//
//    private PathChain dropofftwo;
//
//    public void buildPaths() {
//        scorePreload = new Path(new BezierLine(startPose, scorePose));
//        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
//
//        grabPrePickup1 = follower.pathBuilder()
////                .addPath(
////                        new BezierCurve(
////                                new Pose(96.000, 96.000),
////                                new Pose(67.241, 78.414),
////                                new Pose(117, 83.79310344827586)
////                        )
////                )
//                .addPath(
//                        new BezierCurve(
//                                new Pose(96.000, 96.000),
//                                new Pose(84.74244604316547, 55.32086330935252),
//                                new Pose(128, 53)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//                .build();
//
//        gatepickup = follower.pathBuilder()
//                .addPath(
//                        new BezierCurve(
//                                new Pose(87, 87),
//                                new Pose(42.414, 72.414),
//                                new Pose(131, 56) //130 58
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(25))
//                .build();
//        grabPrePickup2 = follower.pathBuilder()
//                .addPath(
//                        new BezierCurve(
//                                new Pose(122.600, 122.500),
//                                new Pose(66.207, 52.759),
//                                new Pose(129.310, 56.276)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//                .addPath(new BezierLine(prePickup2, scorePose))
//                .setLinearHeadingInterpolation(prePickup2.getHeading(), scorePose.getHeading())
//                .build();
//        scorePickup1 = follower.pathBuilder()
//                .addPath(
//                        new BezierCurve(
//                                new Pose(129.310, 56.276),
//                                new Pose(59.793, 90.207),
//                                new Pose(87, 87)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//                .build();
//        grabPrePickup3 = follower.pathBuilder()
//                .addPath(
//                        new BezierCurve(
//                                new Pose(129, 60),
//                                new Pose(79.35539568345324, 67.5453237410072),
//                                new Pose(87, 87)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(25), Math.toRadians(0))
//                .build();
////        grabPickup2 = follower.pathBuilder()
////
////                .build();
//////        dropofftwo = follower.pathBuilder()
//////                .addPath(new BezierLine(pickup2Pose,dropoff2))
//////                .setLinearHeadingInterpolation(pickup2Pose.getHeading(),dropoff2.getHeading())
//////                .build();
////        scorePickup2 = follower.pathBuilder()
////                .addPath(new BezierLine(pickup2Pose, scorePose))
////                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
////                .build();
////
////        grabPrePickup3 = follower.pathBuilder()
////                .addPath(new BezierLine(scorePose, prePickup3))
////                .setLinearHeadingInterpolation(scorePose.getHeading(), prePickup3.getHeading())
////                .build();
////
////        grabPickup3 = follower.pathBuilder()
////                .addPath(new BezierLine(prePickup3, pickup3Pose))
////                .setLinearHeadingInterpolation(prePickup3.getHeading(), pickup3Pose.getHeading())
////                .build();
////
////        scorePickup3 = follower.pathBuilder()
////                .addPath(new BezierLine(pickup3Pose, scorePose))
////                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
////                .build();
//    }
//
//    public void autonomousPathUpdate() {
//        switch (pathState) {
//            case 0:
//                Hood.INSTANCE.midopen();
//                scheduleOuttake();
//                follower.followPath(scorePreload, true);
//                setPathState(1);
//
//                break;
//
//            case 1:
//                if (!follower.isBusy()) {
//                    turretAlignment.align();
//                    // SHOOT after preload
//                    shootThreeBalls();
//
//                    Intake();
//                    follower.followPath(grabPrePickup1);
//
//                    setPathState(2);
//                }
//                break;
//
//            case 2:
//                if (!follower.isBusy()) {
//                    Transfer.INSTANCE.off();
////                    CompliantIntake.INSTANCE.off();
////                    TurretPID.INSTANCE.setFarShooterSpeed();
//                    secondshotforyouuuuu();
//                    follower.followPath(scorePickup1, true);
//                    setPathState(3);
//                }
//                break;
//
//            case 3:
//                if (!follower.isBusy()) {
//                    turretAlignment.align();
//                    shootThreeBalls();
//                    Intake();
//                    setPathState(4);
//                }
//                break;
//
//            case 4:
//                if (!follower.isBusy()) {
//                    follower.followPath(gatepickup);
//
////                    follower.followPath(grabPrePickup2, true);
////                    Intake.INSTANCE.activeintake.setPower(1);
////                    Midtake.INSTANCE.newtake.setPower(0.3);
//                    setPathState(5);
//                }
//                break;
//
//            case 5:
//                if (!follower.isBusy()) {
//                    GateIntake();
//                    follower.followPath(curvescore);
//                    setPathState(6);
//                }
//                break;
////
//            case 6:
//                if (!follower.isBusy()) {
//                    turretAlignment.align();
//                    secondshotforyouuuuu();
//                    shootThreeBalls();
//                    setPathState(7);
//                }
//                break;
////
////            case 7:
////                if (!follower.isBusy()) {
////                    thirdshootThreeBalls();
////                    follower.followPath(grabPrePickup3, true);
////                    Intake.INSTANCE.activeintake.setPower(1);
////                    Midtake.INSTANCE.newtake.setPower(0.3);
////                    setPathState(8);
////                }
////                break;
////
////            case 8:
////                if (!follower.isBusy()) {
////                    follower.followPath(grabPickup3, true);
////                    setPathState(9);
////                }
////                break;
////
////            case 9:
////                if (!follower.isBusy()) {
////                    Midtake.INSTANCE.newtake.setPower(0);
////                    Intake.INSTANCE.activeintake.setPower(0);
////                    follower.followPath(scorePickup3, true);
//////                    shootThreeBalls();
////                    setPathState(10);
//////                }
////                break;
////
////            case 10:
////                if (!follower.isBusy()) {
////                    secondshootThreeBalls();
////                    setPathState(-1);
////                }
////                break;
//        }
//    }
//
//    private void scheduleOuttake() {
//    TurretPID.INSTANCE.setMidCloseShooterSpeed().schedule();
//    }
//    private void secondshotforyouuuuu() {
//        TurretPID.INSTANCE.setMidCloseShooterSpeed().schedule();
//    }
//    private void Intake() {
//        CompliantIntake.INSTANCE.on();
//        Transfer.INSTANCE.advance();
//    }
//    private void setShooterFromOdometry() {
//        Pose pose = follower.getPose();
//        double distance = Math.hypot(
//                96 - pose.getX(),  // TARGET_X = 96 (your scorePose X)
//                96 - pose.getY()   // TARGET_Y = 96 (your scorePose Y)
//        );
//        TurretPID.INSTANCE.setShooterFromDistance(distance).schedule();
//    }
//    private void GateIntake() {
//        CompliantIntake.INSTANCE.on();
//        Transfer.INSTANCE.advance();
//        sleep(1500);
//        Transfer.INSTANCE.off();
//
//    }
//
//    private void shootThreeBalls() {
//    CompliantIntake.INSTANCE.on();
//    Transfer.INSTANCE.on();
//    sleep(1000);
//        CompliantIntake.INSTANCE.off();
//        Transfer.INSTANCE.off();
//    }
//
//    public void setPathState(int pState) {
//        pathState = pState;
//        pathTimer.resetTimer();
//    }
//
//    @Override
//    public void onUpdate() {
//        follower.update();
//        autonomousPathUpdate();
//        telemetry.addData("Path State", pathState);
//        telemetry.addData("X", follower.getPose().getX());
//        telemetry.addData("Y", follower.getPose().getY());
//        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
//        telemetry.update();
//    }
//
//    @Override
//    public void onInit() {
//        pathTimer = new Timer();
//        opmodeTimer = new Timer();
//        opmodeTimer.resetTimer();
//        volt.init(hardwareMap);
//        limelight = new RedExperimentalDistanceLExtractor(hardwareMap);
//        turretAlignment = new SimpleLL(hardwareMap, limelight);
//        limelight.startReading();
//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(startPose);
//        buildPaths();
//    }
//
//    @Override public void onWaitForStart() {}
//
//    @Override public void onStartButtonPressed() {
//        opmodeTimer.resetTimer();
//        setPathState(0);
//    }
//
//    @Override public void onStop() {
//        Intake.INSTANCE.activeintake.setPower(0);
//        outtake.setPower(0);
//        Midtake.newtake.setPower(0);
//    }
//
//}