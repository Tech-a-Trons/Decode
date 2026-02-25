package org.firstinspires.ftc.teamcode.Season.Auto.RegionalsAuto;


import org.firstinspires.ftc.teamcode.Season.Auto.Constants;
import org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems.RedExperimentalDistanceLExtractor;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.Turret;

import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.CompliantIntake;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.Hood;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.ManualTurret;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.NewHood;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.OuttakePID;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.RedLL;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.RobotContext;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.ShooterPID;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.Transfer;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.TurretOdoAi;
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
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.ftc.components.LoopTimeComponent;

@Autonomous(name = "RedGate18", group = "Examples")
public class RedGate18 extends NextFTCOpMode {
    VoltageGet volt = new VoltageGet();

    public RedGate18() {
        addComponents(
                new SubsystemComponent(NewHood.INSTANCE, CompliantIntake.INSTANCE,Transfer.INSTANCE,ManualTurret.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE, new LoopTimeComponent()
        );
    }
    private VoltageGet voltageGet;

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(123.22766570605188, 123.15850144092221, Math.toRadians(35));
    //    private final Pose scorePose = new Pose(90, 90, Math.toRadians(215));
    private final Pose scorePose = new Pose(85, 85, Math.toRadians(0));

    private final Pose preprePickup1 = new Pose(12, 12, Math.toRadians(180));
    private final Pose prePickup1 = new Pose(17.581, 10.0470, Math.toRadians(180));
    private final Pose prePickup2 = new Pose(126.207, 59.379, Math.toRadians(180)); //55
    private final Pose prePickup3 = new Pose(46.5, 84, Math.toRadians(180));
    //    private final Pose dropoff2 = new Pose(100, 54, Math.toRadians(180)); //55
    private final Pose pickup1Pose = new Pose(126.41379310344828, 87, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(9, 52, Math.toRadians(180));
    private final Pose pickup3Pose = new Pose(12, 84, Math.toRadians(180));

    private Path scorePreload;
    private RedExperimentalDistanceLExtractor limelight;


    private static final double TARGET_X = 121;  // Example: center of field
    private static final double TARGET_Y = 121;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;
    private PathChain grabPrePickup1, grabPrePickup2, grabPrePickup3, gatepickup, curvescore,leave;

    private PathChain dropofftwo;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        grabPrePickup1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(85.000, 85.000),
                                new Pose(95.669, 53.833),
                                new Pose(170, 59.524)
                        )
                )

                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
               .addPath(
          new BezierCurve(
            new Pose(170, 59.524),
            new Pose(94.007, 60.817),
            new Pose(85.000, 85.000)
          )
        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();



        grabPickup3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(85.000, 85.000),

                                new Pose(130, 84.421)
                        )
                ).setTangentHeadingInterpolation()

                .build();
        grabPrePickup2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(85.000, 85.000),
                                new Pose(93.635, 58.775),
                                new Pose(132, 61.300)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(25))
                .build();
        scorePickup1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(123, 84.53525179856116), new Pose(83.000, 83.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        scorePickup2 = follower.pathBuilder()
                .addPath(
                new BezierCurve(
                        new Pose(129, 61.300),
                        new Pose(86.203, 68.473),
                        new Pose(85.000, 85.000)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(25), Math.toRadians(0))
                .build();
//        grabPickup2 = follower.pathBuilder()
//
//                .build();
////        dropofftwo = follower.pathBuilder()
////                .addPath(new BezierLine(pickup2Pose,dropoff2))
////                .setLinearHeadingInterpolation(pickup2Pose.getHeading(),dropoff2.getHeading())
////                .build();
        scorePickup3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(130, 84.421),

                                new Pose(85.000, 85.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
//
        leave = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(85.000, 85.000),

                                new Pose(103.187, 75.752)
                        )
                ).setTangentHeadingInterpolation()
                .build();
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
                NewHood.INSTANCE.midclose();
                ManualTurret.INSTANCE.setPosition(0);
                secondshotforyouuuuu();
                follower.followPath(scorePreload, true);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    // SHOOT after preload
                    shootThreeBalls();

                    Intake();
                    follower.followPath(grabPrePickup1);
                    setShooterFromOdometry();
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    Transfer.INSTANCE.off();
                    shootThreeBalls();
                    Intake();
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(grabPrePickup2);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    GateIntake();
                    follower.followPath(scorePickup2);
                    setShooterFromOdometry();
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    shootThreeBalls();
                    Intake();
                    setPathState(6);
                }
                break;
//
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(grabPrePickup2);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    GateIntake();
                    follower.followPath(scorePickup2);
                    setShooterFromOdometry();
                    setPathState(8);
                }
                break;
//
            case 8:
                if (!follower.isBusy()) {
                    shootThreeBalls();
                    Intake();
                    setPathState(9);
                }
                break;
//
            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(grabPrePickup2);

                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    GateIntake();
                    follower.followPath(scorePickup2);
                    setShooterFromOdometry();
                    setPathState(11);
                }
                break;
//
            case 11:
                if (!follower.isBusy()) {
                    shootThreeBalls();
                    Intake();

//                    shootThreeBalls();

                    setPathState(12);
                }
                break;
            case 12:
            if (!follower.isBusy()) {
                follower.followPath(grabPickup3, true);
                setShooterFromOdometry();

//                    shootThreeBalls();
//                    follower.followPath(grabPickup3, true);
//                    scheduleOuttake();
                setPathState(13);
            }
            break;
            case 13:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup3);
//                    shootThreeBalls();
//                    follower.followPath(leave);
                    setPathState(14);
                }
                break;
            case 14:
                if (!follower.isBusy()) {
                    shootThreeBalls();
                    follower.followPath(leave);
                    setPathState(-1);
                }
                break;
        }
    }
    private void savePose() { // runs when auto finishes
        RobotContext.lastPose = follower.getPose();
    }
    private void scheduleOuttake() {

    }

    private void Intake() {
        CompliantIntake.INSTANCE.on();
        Transfer.INSTANCE.repel();
    }
    private void setShooterFromOdometry() {
        ShooterPID.INSTANCE.setTargetVelocity(1231);
    }

    private void secondshotforyouuuuu() {
        ShooterPID.INSTANCE.setTargetVelocity(1231);
    }

    private void GateIntake() {
        CompliantIntake.INSTANCE.on();
        Transfer.INSTANCE.repel();
        sleep(1600);
        Transfer.INSTANCE.off();

    }

    private void shootThreeBalls() {

            sleep(600);
            CompliantIntake.INSTANCE.on();
            Transfer.INSTANCE.on();
            sleep(900);
            CompliantIntake.INSTANCE.off();
            Transfer.INSTANCE.off();
            ShooterPID.INSTANCE.stop();


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
        telemetry.addData("Velocity", ShooterPID.INSTANCE.getActualVelocity());
        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void onInit() {

ShooterPID.init(hardwareMap);
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        volt.init(hardwareMap);
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