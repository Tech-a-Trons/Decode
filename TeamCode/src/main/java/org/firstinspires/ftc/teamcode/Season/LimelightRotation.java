package org.firstinspires.ftc.teamcode.Season;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Season.SensorStuff.SeasonLimelightExtractor;

@TeleOp
public class LimelightRotation extends LinearOpMode {

    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;

    @Override
    public void runOpMode() {

        SeasonLimelightExtractor ll = new SeasonLimelightExtractor(hardwareMap);

        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");

        // Reverse the right side
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);

        ll.setTelemetry(telemetry);
        ll.startReading();

        telemetry.addLine("rdy!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            ll.update();

            double correction = ll.getSteeringCorrection(0.02); // Tune kP
            boolean visible = ll.isTargetVisible();

            double rPwr = 0.0;

            if (visible) {
                // Rotate to center the target
                rPwr = correction;
            }

            // Mecanum rotation (no translation)
            double flp = rPwr;
            double frp = -rPwr;
            double blp = rPwr;
            double brp = -rPwr;

            fl.setPower(flp);
            fr.setPower(frp);
            bl.setPower(blp);
            br.setPower(brp);

            telemetry.addData("Correction", correction);
            telemetry.addData("Target Visible", visible);
            telemetry.update();

            sleep(20);
        }

        ll.stopReading();
    }
}
