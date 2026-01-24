package org.firstinspires.ftc.teamcode.Season.TeleOp.preQ2PrototypeTeleops;

//import static org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.BlueTurretPID.turret;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.ExperimentalBlueTurretPID;

@Disabled
@TeleOp
public class BluePIDTest extends LinearOpMode {
    //BlueExperimentalDistanceLExtractor ll = new BlueExperimentalDistanceLExtractor(hardwareMap);
    private ExperimentalBlueTurretPID shooter;

    @Override
    public void runOpMode() {

        // Create subsystem AFTER hardwareMap exists
        shooter = new ExperimentalBlueTurretPID(hardwareMap);
//
//        telemetry.addLine("Ready");
//        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Set targets
            if (gamepad1.a) {
                shooter.setCloseShooterSpeed();
            } else if (gamepad1.b) {
                shooter.setMidCloseShooterSpeed();
            } else if (gamepad1.y) {
                shooter.setFarShooterSpeed();
            } else if (gamepad1.x) {
                shooter.stopShooter();
            }

            // 🚨 THIS IS REQUIRED
            shooter.periodic();
//
//            telemetry.addData("Velocity", shooter.getVelocity()); // optional
//            telemetry.update();
        }
    }
}