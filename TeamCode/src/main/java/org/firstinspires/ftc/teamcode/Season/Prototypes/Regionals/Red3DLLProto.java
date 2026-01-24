package org.firstinspires.ftc.teamcode.Season.Prototypes.Regionals;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems.RedLLExtractor3D;

public class Red3DLLProto extends LinearOpMode {
    RedLLExtractor3D extractor3D;
    @Override
    public void runOpMode() throws InterruptedException {
        extractor3D = new RedLLExtractor3D(hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive()) {
            extractor3D.update();
            extractor3D.displayTelemetry();
        }
        extractor3D.stop();
    }
}
