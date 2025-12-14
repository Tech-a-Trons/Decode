package org.firstinspires.ftc.teamcode.Season.TeleOp.NewPrototypeTeleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems.RedExperimentalDistanceLExtractor;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.BlueTurretPID;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.DistanceRedTurretPID;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.LLHood;

public class DistRedPIDTest extends LinearOpMode {
    RedExperimentalDistanceLExtractor ll = new RedExperimentalDistanceLExtractor(hardwareMap);
    LLHood hood = new LLHood(hardwareMap);
    private DistanceRedTurretPID shooter;

    @Override
    public void runOpMode() {

        // Create subsystem AFTER hardwareMap exists
        shooter = new DistanceRedTurretPID(hardwareMap);

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Set targets
            if (gamepad1.a) {
                shooter.setShooterSpeed();
                hood.HoodPower();
            } else if (gamepad1.x) {
                shooter.stopShooter();
            }

            // 🚨 THIS IS REQUIRED
            shooter.periodic();

            telemetry.addData("Velocity", shooter.getVelocity()); // optional
            telemetry.update();
        }
    }
}