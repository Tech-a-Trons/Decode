package org.firstinspires.ftc.teamcode.Season.TeleOp.NewPrototypeTeleops;

import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorGoBildaPinpoint;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp
//This test is meant for our turret when the robot starts from the goal
public class OdoTest extends OpMode {
    GoBildaPinpointDriver pinpoint;
    Servo turret1;
    Servo turret2;

    @Override
    public void init() {
        // Get a reference to the sensor
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        turret1 = hardwareMap.get(Servo.class,"turret1");
        turret2 = hardwareMap.get(Servo.class,"turret2");

        // Configure the sensor
        configurePinpoint();

        // Set the location of the robot - this should be the place you are starting the robot from
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
    }

    @Override
    public void loop() {

        // Goal is fixed at field origin
        double goalX = 0.0;
        double goalY = 0.0;

        pinpoint.update();
        Pose2D pose = pinpoint.getPosition();

        double x = pose.getX(DistanceUnit.INCH);
        double y = pose.getY(DistanceUnit.INCH);

        // Convert heading to radians
        double headingRad = Math.toRadians(
                pose.getHeading(AngleUnit.DEGREES)
        );

        // Vector from robot to goal
        double dx = goalX - x;
        double dy = goalY - y;

        // Field-centric angle to goal
        double fieldAngle = Math.atan2(dy, dx);

        // Turret angle relative to robot
        double turretAngle = fieldAngle - headingRad;

        // Normalize to [-π, π]
        turretAngle = Math.atan2(
                Math.sin(turretAngle),
                Math.cos(turretAngle)
        );

        /*
         * Map turret angle to servo
         * Assumes:
         *   - 0.5 = forward
         *   - ±90° turret travel
         */
        double servoPos = (turretAngle / Math.toRadians(180.0)) + 0.5;

        // Clamp for safety
        servoPos = Math.max(0.0, Math.min(1.0, servoPos));

        turret1.setPosition(servoPos);
        turret2.setPosition(servoPos);

        telemetry.addData("Robot X", x);
        telemetry.addData("Robot Y", y);
        telemetry.addData("Robot Heading (deg)", Math.toDegrees(headingRad));
        telemetry.addData("Turret Angle (deg)", Math.toDegrees(turretAngle));
        telemetry.addData("Servo Pos", servoPos);
    }


    public void configurePinpoint(){
        /*
         *  Set the odometry pod positions relative to the point that you want the position to be measured from.
         *
         *  The X pod offset refers to how far sideways from the tracking point the X (forward) odometry pod is.
         *  Left of the center is a positive number, right of center is a negative number.
         *
         *  The Y pod offset refers to how far forwards from the tracking point the Y (strafe) odometry pod is.
         *  Forward of center is a positive number, backwards is a negative number.
         */
        pinpoint.setOffsets(-84.0, -168.0, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1

        /*
         * Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
         * the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
         * If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
         * number of ticks per unit of your odometry pod.  For example:
         *     pinpoint.setEncoderResolution(13.26291192, DistanceUnit.MM);
         */
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        /*
         * Set the direction that each of the two odometry pods count. The X (forward) pod should
         * increase when you move the robot forward. And the Y (strafe) pod should increase when
         * you move the robot to the left.
         */
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        /*
         * Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
         * The IMU will automatically calibrate when first powered on, but recalibrating before running
         * the robot is a good idea to ensure that the calibration is "good".
         * resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
         * This is recommended before you run your autonomous, as a bad initial calibration can cause
         * an incorrect starting value for x, y, and heading.
         */
        pinpoint.resetPosAndIMU();
    }
}
