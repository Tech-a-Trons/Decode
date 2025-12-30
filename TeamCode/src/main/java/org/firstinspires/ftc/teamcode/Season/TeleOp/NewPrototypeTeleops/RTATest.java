package org.firstinspires.ftc.teamcode.Season.TeleOp.NewPrototypeTeleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems.RedExperimentalDistanceLExtractor;

import org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems.RedTurretAlign;

import dev.nextftc.ftc.NextFTCOpMode;

@TeleOp
public class RTATest extends OpMode {

    RedExperimentalDistanceLExtractor ll;
    RedTurretAlign turretAlign;

    @Override
    public void init() {
        turretAlign = new RedTurretAlign();
        turretAlign.initHardware(hardwareMap);

        ll = new RedExperimentalDistanceLExtractor(hardwareMap);
        ll.setTelemetry(telemetry);

        ll.startReading();
    }

    @Override
    public void loop() {
        ll.update();
        turretAlign.rotate();

        telemetry.addData("ServoPos: ", turretAlign.ServoPos());
        telemetry.update();

    }
}