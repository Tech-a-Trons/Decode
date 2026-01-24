package org.firstinspires.ftc.teamcode.Season.TeleOp.preQ2PrototypeTeleops;

import static java.lang.Math.atan2;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Disabled
@TeleOp(name="NindroidOdoTest")
public class NindroidOdoTest extends LinearOpMode {

    GoBildaPinpointDriver pinpoint;
    Servo turret1, turret2;

    double turretCurrentAngle = 0;

    @Override
    public void runOpMode() {

        // --- Hardware mapping ---
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        turret1 = hardwareMap.get(Servo.class, "turret1");
        turret2 = hardwareMap.get(Servo.class, "turret2");

        // --- Configure Pinpoint ---
        configurePinpoint();
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

        waitForStart();

        while (opModeIsActive()) {

            // --- Update Pinpoint position ---
            if(gamepad1.a){
                pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
            }
            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();

            double robotX = pose.getX(DistanceUnit.INCH);
            double robotY = pose.getY(DistanceUnit.INCH);
            double heading = pose.getHeading(AngleUnit.DEGREES);

            // --- Turret tracking logic ---
            double dx = 0 - robotX; // target X = 0
            double dy = 0 - robotY; // target Y = 0
            double targetAngle = Math.toDegrees(atan2(dy, dx));

            double tangle = targetAngle - heading;
            turret1.setPosition(tangle);
            turret2.setPosition(tangle);
            turretCurrentAngle = tangle;

            // --- Telemetry ---
            telemetry.addData("X coordinate (IN)", robotX);
            telemetry.addData("Y coordinate (IN)", robotY);
            telemetry.addData("Heading angle (DEGREES)", heading);
            telemetry.update();
        }
    }

    private void configurePinpoint(){
        pinpoint.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
    }
}