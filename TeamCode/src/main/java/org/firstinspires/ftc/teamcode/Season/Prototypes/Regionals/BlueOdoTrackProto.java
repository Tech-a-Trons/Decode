package org.firstinspires.ftc.teamcode.Season.Prototypes.Regionals;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems.OdoTrackBlue;
import org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems.OdoTrackRed;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems.O;

@TeleOp
public class BlueOdoTrackProto extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Initialize at bottom-left starting position
        OdoTrackBlue extractor = new OdoTrackBlue(
                hardwareMap,
                telemetry,
                70.5,   // 12" from left wall
                145.5,   // 12" from bottom wall
                0.0     // facing forward
        );

        //extractor.disableAutoInit();  // ← ADD THIS LINE!
        telemetry.addLine("========== INITIALIZATION CHECK ==========");
        telemetry.addData("Position should be", "70.5, 145.5 @ 0");
        telemetry.addData("Actual position", "%.1f, %.1f @ %.1f°",
                extractor.getX(), extractor.getY(), extractor.getHeading());
        telemetry.addData("Distance to goal", "%.1f", extractor.getDistanceToGoal());
        telemetry.addData("Expected distance", "~81.4 inches");
        telemetry.addLine("If distance is 171.1, position is WRONG!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.update();
            // Update every loop
            extractor.update();

            // Display info
            extractor.displayTelemetry();
            telemetry.update();

            // Use the data
//            double distance = extractor.getDistanceToGoal();
//            double angle = extractor.getAngleToGoal();
//
//            telemetry.addData("To Goal", "%.1f inches @ %.1f°", distance, angle);
//            telemetry.update();
        }

        extractor.stop();
    }
}