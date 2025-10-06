package org.firstinspires.ftc.teamcode.Season;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

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

        Double tx = ll.getTx();
        Double ty = ll.getTy();
        Double ta = ll.getTa();

        telemetry.addLine("Ready!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            ll.update();

            if (tx > 1.0) {
                fl.setPower(0.05);
                fr.setPower(-0.05);
                bl.setPower(0.05);
                br.setPower(-0.05);
            } else if (tx < -1.0) {
                fl.setPower(-0.05);
                fr.setPower(0.05);
                bl.setPower(-0.05);
                br.setPower(0.05);
            } else if (tx < 1.0 && tx > -1.0) {
                fl.setPower(0.0);
                fr.setPower(0.0);
                bl.setPower(0.0);
                br.setPower(0.0);
            } else if (tx == 0.0 && ty == 0.0 && ta == 0.0) {
                telemetry.addLine("Not detected!");
                telemetry.update();
                fl.setPower(0.0);
                fr.setPower(0.0);
                bl.setPower(0.0);
                br.setPower(0.0);
            } else {
                return;
            }
        }
        ll.stopReading();
    }
}
