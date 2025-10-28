package org.firstinspires.ftc.teamcode.Season.Prototypes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems.StableSeasonLExtractor;

//Example Limelight rotation code

@Disabled
@TeleOp
public class LimelightRotation extends LinearOpMode {

    DcMotor fl = null;
    DcMotor fr = null;
    DcMotor bl = null;
    DcMotor br = null;

    @Override
    public void runOpMode() {

        StableSeasonLExtractor ll = new StableSeasonLExtractor(hardwareMap);

        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");

        // Reverse the left side
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);

        ll.setTelemetry(telemetry);
        ll.startReading();

        Double tx = ll.getTx();
        if (tx == null) {
            tx = 0.0;
        }

        telemetry.addLine("Ready!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            ll.update();
            tx = ll.getTx();
            if (tx == null) {
                tx = 0.0;
            }

            if (tx > 1.0) {
                fl.setPower(0.15);
                fr.setPower(-0.15);
                bl.setPower(0.15);
                br.setPower(-0.15);
            } else if (tx < -1.0) {
                fl.setPower(-0.15);
                fr.setPower(0.15);
                bl.setPower(-0.15);
                br.setPower(0.15);
            } else if (tx < 1.0 && tx > -1.0) {
                fl.setPower(0.0);
                fr.setPower(0.0);
                bl.setPower(0.0);
                br.setPower(0.0);
                break;
            } else {
                telemetry.addLine("JUNK");
            }
            ll.update();
        }
        ll.stopReading();
    }
}
