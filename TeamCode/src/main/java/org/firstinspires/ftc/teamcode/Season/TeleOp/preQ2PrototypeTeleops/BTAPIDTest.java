package org.firstinspires.ftc.teamcode.Season.TeleOp.preQ2PrototypeTeleops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems.BlueExperimentalDistanceLExtractor;
import org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems.BlueTurretAlign;

import dev.nextftc.ftc.NextFTCOpMode;

@Disabled
@TeleOp
public class BTAPIDTest extends NextFTCOpMode {
    @Override
    public void onStartButtonPressed() {

        BlueTurretAlign turretAlign = BlueTurretAlign.INSTANCE;
        turretAlign.initHardware(hardwareMap);

        BlueExperimentalDistanceLExtractor ll = new BlueExperimentalDistanceLExtractor(hardwareMap);
        turretAlign.setLimelight(ll);

        ll.setTelemetry(telemetry);

        // Enable ALWAYS-ON alignment
        turretAlign.setAlignmentActive(true);
    }
}