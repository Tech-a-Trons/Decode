package org.firstinspires.ftc.teamcode.Season.TeleOp.NewPrototypeTeleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems.CRTA;
import org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems.RedExperimentalDistanceLExtractor;

import org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems.RedTurretAlign;

import dev.nextftc.ftc.NextFTCOpMode;

@TeleOp
public class CRTAtest extends OpMode {

    RedExperimentalDistanceLExtractor ll;
    CRTA turretAlign;

    @Override
    public void init() {
        turretAlign = new CRTA();
        turretAlign.initHardware(hardwareMap);

        ll = new RedExperimentalDistanceLExtractor(hardwareMap);
        ll.setTelemetry(telemetry);
    }

    @Override
    public void loop() {
        ll.update();
        turretAlign.rotate();

        telemetry.addData("Servo Pwr: ", turretAlign.ServoPower());
        telemetry.update();

    }
}