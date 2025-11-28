package org.firstinspires.ftc.teamcode.Season.Pedro.oldauto;

import static org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Outtake.outtake;

import org.firstinspires.ftc.teamcode.Season.Pedro.Constants;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Intake;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Midtake;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Outtake;
import org.firstinspires.ftc.teamcode.Season.Subsystems.Sensors.VoltageGet;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
@Disabled
@Autonomous(name = "Lebron", group = "Examples")
public class Lebron extends NextFTCOpMode {

    VoltageGet volt = new VoltageGet();

    public Lebron() {
        addComponents(
                new SubsystemComponent(Outtake.INSTANCE, Intake.INSTANCE, Midtake.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    private PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7;

    private final Pose startPose = new Pose(128.093, 127.674, Math.toRadians(220));

    public void buildPaths() {
        // --- PATH 1 ---
        Path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(128.093, 127.674),
                        new Pose(88.000, 88.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(220), Math.toRadians(220))
                .build();

        // --- PATH 2 ---
        Path2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(88.000, 88.000),
                        new Pose(60.279, 69.488),
                        new Pose(125, 86.233)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(220), Math.toRadians(0))
                .build();

        // --- PATH 3 ---
        Path3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(125, 86.233),
                        new Pose(87.907, 88.116)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(220))
                .build();

        // --- PATH 4 ---
        Path4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(87.907, 88.116),
                        new Pose(72.209, 51.279),
                        new Pose(125, 59.442)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(220), Math.toRadians(0))
                .build();

        // --- PATH 5 ---
        Path5 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(125, 59.442),
                        new Pose(85.186, 64.465),
                        new Pose(87.907, 88.326)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(220))
                .build();

        // --- PATH 6 ---
        Path6 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(87.907, 88.326),
                        new Pose(81.837, 28.047),
                        new Pose(125, 36.209)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(220), Math.toRadians(0))
                .build();

        // --- PATH 7 ---
        Path7 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(125, 36.209),
                        new Pose(87.907, 88.535)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(220))
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(Path1);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    shootThreeBalls();
                    follower.followPath(Path2, true);
                    Intake.INSTANCE.activeintake.setPower(1);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(Path3, true);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    Intake.INSTANCE.activeintake.setPower(0);
                    Outtake.INSTANCE.outtake.setPower(0);
                    follower.followPath(Path4, true);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    secondshootThreeBalls();
                    follower.followPath(Path5, true);
                    Intake.INSTANCE.activeintake.setPower(1);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(Path6, true);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    Intake.INSTANCE.activeintake.setPower(0);
                    follower.followPath(Path7, true);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    secondshootThreeBalls();
                    setPathState(-1);
                }
                break;
        }
    }

    private void shootThreeBalls() {
        Outtake outtake = Outtake.INSTANCE;
        Midtake midtake = Midtake.INSTANCE;
        Intake intake = Intake.INSTANCE;

        outtake.outtake.setPower(volt.regulate(0.44));
        sleep(1000);
        midtake.newtake.setPower(volt.regulate(-1.0));
        sleep(100);
        intake.activeintake.setPower(volt.regulate(1.0));
        sleep(1000);

        // stop
        outtake.outtake.setPower(0);
        midtake.newtake.setPower(0);
        intake.activeintake.setPower(0);
    }

    private void secondshootThreeBalls() {
        Outtake outtake = Outtake.INSTANCE;
        Midtake midtake = Midtake.INSTANCE;
        Intake intake = Intake.INSTANCE;

        outtake.outtake.setPower(volt.regulate(0.46));
        sleep(800);
        midtake.newtake.setPower(volt.regulate(-1));
        sleep(600);
        intake.activeintake.setPower(volt.regulate(1));
        sleep(800);

        outtake.outtake.setPower(0);
        midtake.newtake.setPower(0);
        intake.activeintake.setPower(0);
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