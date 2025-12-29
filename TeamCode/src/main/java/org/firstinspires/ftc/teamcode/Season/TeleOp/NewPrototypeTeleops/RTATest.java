package org.firstinspires.ftc.teamcode.Season.TeleOp.NewPrototypeTeleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems.RedExperimentalDistanceLExtractor;

import org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems.RedTurretAlign;

import dev.nextftc.ftc.NextFTCOpMode;

@TeleOp
public class RTATest extends NextFTCOpMode {

    RedExperimentalDistanceLExtractor ll;
    RedTurretAlign turretAlign;

    @Override
    public void onStartButtonPressed() {
        ll = new RedExperimentalDistanceLExtractor(hardwareMap);
        ll.startReading();
        ll.update();

        turretAlign = new RedTurretAlign();
        turretAlign.initHardware(hardwareMap);

        ll.setTelemetry(telemetry);

        turretAlign.rotate();
    }

    @Override
    public void onUpdate() {
        ll.update();
        telemetry.update();
    }
}