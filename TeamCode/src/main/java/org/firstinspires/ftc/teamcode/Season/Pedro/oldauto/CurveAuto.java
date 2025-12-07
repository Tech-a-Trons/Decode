package org.firstinspires.ftc.teamcode.Season.Pedro.oldauto;

import org.firstinspires.ftc.teamcode.Season.Pedro.Constants;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual1Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual1Subsystem.Midtake;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual1Subsystem.Outtake;
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
@Autonomous(name = "CurveAuto", group = "Examples")
public class CurveAuto extends NextFTCOpMode {
    VoltageGet volt = new VoltageGet();

    public CurveAuto() {
        addComponents(
                new SubsystemComponent(Outtake.INSTANCE, Intake.INSTANCE, Midtake.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    private Paths paths; // New paths container

    @Override
    public void onInit() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        volt.init(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(123.130, 122.087, Math.toRadians(220)));

        paths = new Paths(follower); // Initialize new paths
    }

    @Override
    public void onStartButtonPressed() {
        opmodeTimer.resetTimer();
        setPathState(0); // Start autonomous
        follower.followPath(paths.Preload); // Start with preload path
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
    public void onStop() {
        Intake.INSTANCE.activeintake.setPower(0);
        Outtake.INSTANCE.outtake.setPower(0);
        Midtake.INSTANCE.newtake.setPower(0);
    }

    // 🔹 Autonomous path + subsystem logic
    public void autonomousPathUpdate() {
        if (!follower.isBusy()) {
            switch (pathState) {
                case 0:
                    follower.followPath(paths.Preload, true);
                    setPathState(1);
                    break;
                case 1:
                    shootThreeBalls();
                    follower.followPath(paths.Intake1, true);
                    setPathState(2);
                    break;
                case 2:
                    Intake.INSTANCE.activeintake.setPower(1);
                    Midtake.INSTANCE.newtake.setPower(0.3);
                    follower.followPath(paths.Shoot1, true);
                    setPathState(3);
                    break;
                case 3:
                    Intake.INSTANCE.activeintake.setPower(0);
                    secondshootThreeBalls();
                    Intake.INSTANCE.activeintake.setPower(1);
                    Midtake.INSTANCE.newtake.setPower(0.3);
                    follower.followPath(paths.Intake2, true);
                    setPathState(4);
                    break;
                case 4:
                    Intake.INSTANCE.activeintake.setPower(0);
                    Midtake.INSTANCE.newtake.setPower(0);
                    follower.followPath(paths.Shoot2, true);
                    secondshootThreeBalls();
                    setPathState(5);
                    break;
                case 5:
                    Intake.INSTANCE.activeintake.setPower(1);
                    Midtake.INSTANCE.newtake.setPower(0.3);
                    follower.followPath(paths.Intake3, true);
                    setPathState(6);
                    break;
                case 6:
                    Intake.INSTANCE.activeintake.setPower(0);
                    Midtake.INSTANCE.newtake.setPower(0);
                    follower.followPath(paths.Shoot3, true);
                    secondshootThreeBalls();
                    setPathState(7);
                    break;
                case 7:
                    pathState++; // All paths done
                    break;
            }
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    // 🔹 Shooting / Intake methods
    private void shootThreeBalls() {
        Outtake outtake = Outtake.INSTANCE;
        Midtake midtake = Midtake.INSTANCE;
        Intake intake = Intake.INSTANCE;

        outtake.outtake.setPower(volt.regulate(0.42));
        sleep(800);
        midtake.newtake.setPower(volt.regulate(-1.0));
        sleep(50);
        intake.activeintake.setPower(volt.regulate(1.0));
        midtake.newtake.setPower(volt.regulate(0));
        sleep(50);
        intake.activeintake.setPower(volt.regulate(0));
        outtake.outtake.setPower(volt.regulate(0.41));
        sleep(100);
        midtake.newtake.setPower(volt.regulate(-1));
        sleep(50);
        outtake.outtake.setPower(volt.regulate(0.43));
        sleep(100);
        intake.activeintake.setPower(volt.regulate(1.0));
        midtake.newtake.setPower(volt.regulate(-1));
        sleep(1100);
        outtake.outtake.setPower(0);
        midtake.newtake.setPower(0);
        intake.activeintake.setPower(0);
    }

    private void secondshootThreeBalls() {
        Outtake outtake = Outtake.INSTANCE;
        Midtake midtake = Midtake.INSTANCE;
        Intake intake = Intake.INSTANCE;

        outtake.outtake.setPower(volt.regulate(0.42));
        sleep(800);
        midtake.newtake.setPower(volt.regulate(-1.0));
        sleep(50);
        intake.activeintake.setPower(volt.regulate(1.0));
        midtake.newtake.setPower(volt.regulate(0));
        sleep(100);
        intake.activeintake.setPower(0);
        outtake.outtake.setPower(volt.regulate(0.42));
        sleep(50);
        midtake.newtake.setPower(volt.regulate(-1));
        sleep(50);
        outtake.outtake.setPower(volt.regulate(0.44));
        sleep(100);
        intake.activeintake.setPower(volt.regulate(1.0));
        midtake.newtake.setPower(volt.regulate(-1));
        sleep(1100);
        outtake.outtake.setPower(0);
        midtake.newtake.setPower(0);
        intake.activeintake.setPower(0);
    }

    // 🔹 Inner class for new paths
    public static class Paths {
        public PathChain Preload, Intake1, Shoot1, Intake2, Shoot2, Intake3, Shoot3;

        public Paths(Follower follower) {
            Preload = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(123.130, 122.087), new Pose(95.374, 94.748)))
                    .setLinearHeadingInterpolation(Math.toRadians(220), Math.toRadians(230))
                    .build();

            Intake1 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(95.374, 94.748),
                            new Pose(40.395, 68.023),
                            new Pose(120.000, 83.721)))
                    .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(0))
                    .build();

            Shoot1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(120.000, 83.721), new Pose(95.583, 94.539)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(230))
                    .build();

            Intake2 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(95.583, 94.539),
                            new Pose(48.349, 52.326),
                            new Pose(123.000, 59.270)))
                    .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(0))
                    .build();

            Shoot2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(123.000, 59.270), new Pose(95.583, 94.957)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(230))
                    .build();

            Intake3 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(95.583, 94.957),
                            new Pose(68.651, 25.116),
                            new Pose(123.000, 35.687)))
                    .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(0))
                    .build();

            Shoot3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(123.000, 35.687), new Pose(95.165, 94.539)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(230))
                    .build();
        }
    }
}