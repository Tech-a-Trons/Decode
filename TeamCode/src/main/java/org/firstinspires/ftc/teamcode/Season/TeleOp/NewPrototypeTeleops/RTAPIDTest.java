package org.firstinspires.ftc.teamcode.Season.TeleOp.NewPrototypeTeleops;

import static org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.TurretPID.turret;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems.BlueExperimentalDistanceLExtractor;
import org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems.RedExperimentalDistanceLExtractor;

import org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems.RedTurretAlign;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.Hood;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.TurretPID;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp
public class RTAPIDTest extends NextFTCOpMode {
    RedExperimentalDistanceLExtractor ll;
    RedTurretAlign turretAlign;
    @Override
    public void onStartButtonPressed() {
        ll = new RedExperimentalDistanceLExtractor(hardwareMap);
        ll.startReading();
        ll.update();

        turretAlign = RedTurretAlign.INSTANCE;
        turretAlign.initHardware(hardwareMap);

        turretAlign.setLimelight(ll);

        ll.setTelemetry(telemetry);

        // Enable ALWAYS-ON alignment
        turretAlign.setAlignmentActive(true);
    }
    @Override
    public void onUpdate() {
        ll.update();
        //turretAlign.periodic();
    }
}