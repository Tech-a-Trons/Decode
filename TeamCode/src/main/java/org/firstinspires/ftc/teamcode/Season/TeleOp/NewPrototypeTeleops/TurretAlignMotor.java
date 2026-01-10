package org.firstinspires.ftc.teamcode.Season.TeleOp.NewPrototypeTeleops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems.BlueExperimentalDistanceLExtractor;
import org.firstinspires.ftc.teamcode.Season.Subsystems.Sensors.VoltageGet;

@Disabled
@TeleOp
public class TurretAlignMotor extends LinearOpMode {
    VoltageGet volt = new VoltageGet();
    DcMotor turret;

    //Update Values
    private final double SANGLE_TOLERANCE = -1.8;
    private final double FANGLE_TOLERANCE = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        turret = hardwareMap.get(DcMotor.class,"turret");

        volt.init(hardwareMap);

        BlueExperimentalDistanceLExtractor ll = new BlueExperimentalDistanceLExtractor(hardwareMap);

        Double tx = ll.getTx();
        if (tx == null) {
            tx = 0.0;
        }
        Double distance = ll.getEuclideanDistance();
        if (distance == null) {
            distance = 0.0;
        }

        waitForStart();

        ll.update();

        distance = ll.getEuclideanDistance();
        tx = ll.getTx();
        if (tx == null) {
            tx = 0.0;
        }
        if (distance == null) {
            distance = 0.0;
        }

        double sangleError = tx;
        double fangleError = tx;

        if (gamepad1.a) {
            if (Math.abs(sangleError) <= SANGLE_TOLERANCE) {
                turret.setPower(0);
            } else {
                while ( Math.abs(sangleError) >= SANGLE_TOLERANCE) {
                    turret.setPower(0.4);
                    ll.update();
                }
            }
        }

        if (gamepad1.b) {
            if (Math.abs(fangleError) <= FANGLE_TOLERANCE) {
                turret.setPower(0);
            } else {
                while (Math.abs(fangleError) >= FANGLE_TOLERANCE) {
                    turret.setPower(0.4);
                    ll.update();
                }
            }
        }
    }
}
