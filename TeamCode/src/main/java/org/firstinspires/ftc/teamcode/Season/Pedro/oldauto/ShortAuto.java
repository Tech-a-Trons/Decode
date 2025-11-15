//package org.firstinspires.ftc.teamcode.Season.Pedro.oldauto;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.teamcode.Season.Pedro.Constants;
//
//import dev.nextftc.ftc.NextFTCOpMode;
//
//@Autonomous(name="Beziershiii")
//public class ShortAuto extends NextFTCOpMode {
//    Servo claw;
//    private Follower follower;
//    private Paths paths;
//
//    private int pathState = 1; // Start with Path1
//
//    @Override
//    public void init() {
//        follower = Constants.createFollower(hardwareMap);
//        follower.setPose(new Pose(123.130, 122.087, Math.toRadians(220)));
//
//        // Create paths
//        paths = new Paths(follower);
//    }
//
//    @Override
//    public void start() {
//        // Start with Path1
//        follower.followPath(paths.Preload);
//    }
//
//    @Override
//    public void loop() {
//        follower.update();
//
//        // Progress through paths
//        if (!follower.isBusy()) {
//            switch (pathState) {
//                case 1:
//                    follower.followPath(paths.Intake1);
//                    pathState++;
//                    break;
//                case 2:
//                    follower.followPath(paths.Shoot1);
//                    pathState++;
//                    break;
//                case 3:
//                    follower.followPath(paths.Intake2);
//                    pathState++;
//                    break;
//                case 4:
//                    follower.followPath(paths.Shoot2);
//                    pathState++;
//                    break;
//                case 5:
//                    follower.followPath(paths.Intake3);
//                    pathState++;
//                    break;
//                case 6:
//                    follower.followPath(paths.Shoot3);
//                    pathState++;
//                    break;
//                case 7:
//                    // All paths completed
//                    pathState++;
//                    break;
//            }
//        }
//
//        telemetry.addData("Pose", follower.getPose());
//        telemetry.addData("Path State", pathState);
//        telemetry.update();
//    }
//
//    // 🔹 Inner class for paths
//    public static class Paths {
//
//        public PathChain Preload;
//        public PathChain Intake1;
//        public PathChain Shoot1;
//        public PathChain Intake2;
//        public PathChain Shoot2;
//        public PathChain Intake3;
//        public PathChain Shoot3;
//
//        public Paths(Follower follower) {
//            Preload = follower.pathBuilder()
//                    .addPath(new BezierLine(new Pose(123.130, 122.087), new Pose(95.374, 94.748)))
//                    .setLinearHeadingInterpolation(Math.toRadians(220), Math.toRadians(230))
//                    .build();
//
//            Intake1 = follower.pathBuilder()
//                    .addPath(new BezierCurve(
//                            new Pose(95.374, 94.748),
//                            new Pose(40.395, 68.023),
//                            new Pose(120.000, 83.721)
//                    ))
//                    .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(0))
//                    .build();
//
//            Shoot1 = follower.pathBuilder()
//                    .addPath(new BezierLine(new Pose(120.000, 83.721), new Pose(95.583, 94.539)))
//                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(230))
//                    .build();
//
//            Intake2 = follower.pathBuilder()
//                    .addPath(new BezierCurve(
//                            new Pose(95.583, 94.539),
//                            new Pose(48.349, 52.326),
//                            new Pose(123.000, 59.270)
//                    ))
//                    .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(0))
//                    .build();
//
//            Shoot2 = follower.pathBuilder()
//                    .addPath(new BezierLine(new Pose(123.000, 59.270), new Pose(95.583, 94.957)))
//                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(230))
//                    .build();
//
//            Intake3 = follower.pathBuilder()
//                    .addPath(new BezierCurve(
//                            new Pose(95.583, 94.957),
//                            new Pose(68.651, 25.116),
//                            new Pose(123.000, 35.687)
//                    ))
//                    .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(0))
//                    .build();
//
//            Shoot3 = follower.pathBuilder()
//                    .addPath(new BezierLine(new Pose(123.000, 35.687), new Pose(95.165, 94.539)))
//                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(230))
//                    .build();
//        }
//    }
//}