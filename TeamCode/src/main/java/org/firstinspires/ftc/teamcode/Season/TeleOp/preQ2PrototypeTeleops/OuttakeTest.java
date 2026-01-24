package org.firstinspires.ftc.teamcode.Season.TeleOp.preQ2PrototypeTeleops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.ShooterPIDF;

@Disabled
@TeleOp
public class OuttakeTest extends LinearOpMode {

    DcMotor outleft;
    DcMotor outright;
    CRServo hood;
    CRServo turret;

    ElapsedTime turretTimer = new ElapsedTime();
    boolean turretMoving = false;
    double turretDuration = 1.0;
    double turretPower = 0.0;
    private ShooterPIDF shooter;
    @Override
    public void runOpMode() {

//        outright = hardwareMap.get(DcMotor.class,"outtakeright");
//        outleft = hardwareMap.get(DcMotor.class,"outtakeleft");
        hood = hardwareMap.get(CRServo.class,"hood");
        turret = hardwareMap.get(CRServo.class,"turret");
        shooter = new ShooterPIDF(hardwareMap, "outtakeleft", "outtakeright");

//
//        outleft.setDirection(DcMotorSimple.Direction.FORWARD);
//        outright.setDirection(DcMotorSimple.Direction.REVERSE);
        hood.setDirection(CRServo.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            if (!turretMoving && gamepad1.right_bumper) {
                turretPower = 0.3;
                turret.setPower(turretPower);
                turretTimer.reset();
                turretMoving = true;
            }

            if (!turretMoving && gamepad1.left_bumper) {
                turretPower = -0.3;
                turret.setPower(turretPower);
                turretTimer.reset();
                turretMoving = true;
            }

            if (turretMoving && turretTimer.seconds() >= turretDuration) {
                turret.setPower(0);
                turretMoving = false;
            }

//            if (gamepad1.b) {
////                outright.setPower(0.6);
////                outleft.setPower(0.6);
//            }

//            if (gamepad1.a) {
////                outright.setPower(0.72);
////                outleft.setPower(0.72);
//            }
            if (gamepad1.a) {
                shooter.setTargetVelocity(800); // example ticks/sec
            }
            if (gamepad1.b) {
                shooter.setTargetVelocity(1000); // example ticks/sec
            }

            if (gamepad1.y) {
                hood.setPower(0.3);
            }

            if (gamepad1.dpad_left) {
                hood.setPower(-0.5);
            }

            if (gamepad1.x) {
//                outleft.setPower(0);
//                outright.setPower(0);
                shooter.setTargetVelocity(0);
                hood.setPower(-1);
            }
        }
    }
}