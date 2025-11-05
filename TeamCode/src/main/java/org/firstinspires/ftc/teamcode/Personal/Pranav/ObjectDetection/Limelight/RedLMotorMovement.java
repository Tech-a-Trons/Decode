package org.firstinspires.ftc.teamcode.Personal.Pranav.ObjectDetection.Limelight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Disabled
@TeleOp

public class RedLMotorMovement extends LinearOpMode {

    private Limelight3A limelight;
    private IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {

        limelight = hardwareMap.get(Limelight3A.class, "Limelight");

        telemetry.setMsTransmissionInterval(10);

        limelight.pipelineSwitch(1);

        /*
         * Starts polling for data.
         */
        limelight.start();

        while (opModeIsActive()) {
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
            LLResult result = limelight.getLatestResult();
            telemetry.addData("Latest: ",result);
            telemetry.addData("Orientation: ",orientation);
            if (result != null) {
                if (result.isValid()) {
                    Pose3D botpose = result.getBotpose_MT2();
                    // Use botpose data
                    telemetry.addData("Botpose: ",botpose);
                }
            }
        }
    }
}