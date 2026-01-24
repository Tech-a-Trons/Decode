package org.firstinspires.ftc.teamcode.Season.TeleOp.preQ2PrototypeTeleops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems.CRTA;
import org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems.RedExperimentalDistanceLExtractor;

@Disabled
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

        ll.startReading();
    }

    @Override
    public void loop() {
        ll.update();
        turretAlign.rotate();

        telemetry.addData("Servo Pwr: ", turretAlign.ServoPower());
        telemetry.addData("Starting Pwr: ", turretAlign.StartingPower());
        telemetry.addData("Encoder Voltage: ", turretAlign.EncoderVoltage());
        telemetry.addData("Applied Pwr:  ", turretAlign.AppliedPwr());
        telemetry.addData("Error Val:  ", turretAlign.Error());
        telemetry.addData("Integral: ", turretAlign.getIntegral());
        telemetry.addData("Derivative: ", turretAlign.getDerivative());
        telemetry.update();
    }
}