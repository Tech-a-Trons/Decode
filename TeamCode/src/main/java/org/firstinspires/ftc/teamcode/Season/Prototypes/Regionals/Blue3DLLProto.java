package org.firstinspires.ftc.teamcode.Season.Prototypes.Regionals;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems.BlueLLExtractor3D;
@TeleOp
public class Blue3DLLProto extends LinearOpMode {
    BlueLLExtractor3D extractor3D;
    @Override
    public void runOpMode() throws InterruptedException {
        extractor3D = new BlueLLExtractor3D(hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive()) {
            extractor3D.update();
            extractor3D.displayTelemetry();
        }
        extractor3D.stop();
    }
}
