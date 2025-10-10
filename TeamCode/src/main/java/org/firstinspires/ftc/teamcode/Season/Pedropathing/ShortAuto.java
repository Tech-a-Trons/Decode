package org.firstinspires.ftc.teamcode.Season.Pedropathing;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Path1Auto")
public class ShortAuto extends OpMode {
    Servo claw;



    private Follower follower;

    public static PathBuilder builder;



    public static PathChain pathChain = builder
            .addPath(
                    // Path 1
                    new BezierLine(new Pose(15.000, 128.000), new Pose(42.550, 100.821))
            )
            .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(135))
            .addPath(
                    // Path 2
                    new BezierCurve(
                            new Pose(42.550, 100.821),
                            new Pose(57.852, 80.070),
                            new Pose(16.559, 83.633)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
            .addPath(
                    // Path 3
                    new BezierLine(new Pose(16.559, 83.633), new Pose(42.550, 100.821))
            )
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
            .addPath(
                    // Path 4
                    new BezierCurve(
                            new Pose(42.550, 100.821),
                            new Pose(55.755, 54.498),
                            new Pose(16.559, 59.528)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
            .addPath(
                    // Path 5
                    new BezierLine(new Pose(16.559, 59.528), new Pose(42.550, 100.821))
            )
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
            .addPath(
                    // Path 6
                    new BezierCurve(
                            new Pose(42.550, 100.821),
                            new Pose(63.092, 25.153),
                            new Pose(15.721, 35.214)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
            .addPath(
                    // Path 7
                    new BezierLine(new Pose(15.721, 35.214), new Pose(42.550, 100.821))
            )
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
            .build();
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        builder = follower.pathBuilder();
        follower.setPose(new Pose(8, 80, Math.toRadians(0)));
    }

    @Override
    public void start() {
        follower.followPath(pathChain);

    }

    @Override
    public void loop() {
        follower.update();

        telemetry.addData("Pose", follower.getPose());
        telemetry.update();
    }
}