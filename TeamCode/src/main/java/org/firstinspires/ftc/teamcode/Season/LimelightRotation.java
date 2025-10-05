package org.firstinspires.ftc.teamcode.Season;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class LimelightRotation extends LinearOpMode {

    public Limelight3A limelight;
    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;

    @Override
    public void runOpMode() throws InterruptedException {

        SeasonLimelightExtractor ll = new SeasonLimelightExtractor();

        limelight = hardwareMap.get(Limelight3A.class,"Limelight");

        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");

        limelight.pipelineSwitch(1);

        Double tx = ll.getTx();
        if (tx == null) tx = 0.0;

        // Telemetry-safe: use fallback text if null
        telemetry.addData("tx", tx != null ? String.format("%.2f", tx) : "N/A");

        telemetry.addLine("Connecting to Limelight...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (tx > 1) {
                fl.setPower(0.1);
                fr.setPower(-0.1);
                bl.setPower(0.1);
                br.setPower(-0.1);
            } else if (tx < -1) {
                fl.setPower(-0.1);
                fr.setPower(0.1);
                bl.setPower(-0.1);
                br.setPower(0.1);
            } else if (1 <= tx && tx >= -1) {
                fl.setPower(0);
                fr.setPower(0);
                bl.setPower(0);
                br.setPower(0);
            }
        }
        ll.stop();
    }
}
