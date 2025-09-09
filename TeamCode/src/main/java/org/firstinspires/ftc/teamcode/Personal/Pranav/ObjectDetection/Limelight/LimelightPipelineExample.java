package org.firstinspires.ftc.teamcode.Personal.Pranav.ObjectDetection.Limelight;

import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp (name = "LEPipeline")

public class LimelightPipelineExample extends OpMode {
    Limelight3A limelight;
    IMU imu;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
    }

    @Override
    public void loop() {
        limelight.pipelineSwitch(0); // Switch to pipeline number 0

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double tx = result.getTx(); // How far left or right the target is (degrees)
            double ty = result.getTy(); // How far up or down the target is (degrees)
            double ta = result.getTa(); // How big the target looks (0%-100% of the image)

            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);
            telemetry.addData("Target Area", ta);
        } else {
            telemetry.addData("Limelight", "No Targets");
        }

        result.getPipelineIndex();

        // Sending numbers to Python
        double[] inputs = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0};
        limelight.updatePythonInputs(inputs);

        // Getting numbers from Python
        double[] pythonOutputs = result.getPythonOutput();
        if (pythonOutputs != null && pythonOutputs.length > 0) {
            double firstOutput = pythonOutputs[0];
            telemetry.addData("Python output:", firstOutput);
        }

        // First, tell Limelight which way your robot is facing
//        double robotYaw = imu.g().firstAngle;
//        limelight.updateRobotOrientation(robotYaw);
        if (result != null && result.isValid()) {
            Pose3D botpose_mt2 = result.getBotpose_MT2();
            if (botpose_mt2 != null) {
                double x = botpose_mt2.getPosition().x;
                double y = botpose_mt2.getPosition().y;
                telemetry.addData("MT2 Location:", "(" + x + ", " + y + ")");
            }
        }

        List<LLResultTypes.ColorResult> colorTargets = result.getColorResults();
        for (LLResultTypes.ColorResult colorTarget : colorTargets) {
            LLResultTypes.ColorResult colordetection = null;
            double x = colordetection.getTargetXDegrees(); // Where it is (left-right)
            double y = colordetection.getTargetYDegrees(); // Where it is (up-down)
            double area = colorTarget.getTargetArea(); // size (0-100)
            telemetry.addData("Color Target", "takes up " + area + "% of the image");
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            fiducial.getRobotPoseTargetSpace(); // Robot pose relative it the AprilTag Coordinate System (Most Useful)
            fiducial.getCameraPoseTargetSpace(); // Camera pose relative to the AprilTag (useful)
            fiducial.getRobotPoseFieldSpace(); // Robot pose in the field coordinate system based on this tag alone (useful)
            fiducial.getTargetPoseCameraSpace(); // AprilTag pose in the camera's coordinate system (not very useful)
            fiducial.getTargetPoseRobotSpace(); // AprilTag pose in the robot's coordinate system (not very useful)

            int id = fiducial.getFiducialId(); // The ID number of the fiducial
            LLResultTypes.FiducialResult detection = null;
            double x = detection.getTargetXDegrees(); // Where it is (left-right)
            double y = detection.getTargetYDegrees(); // Where it is (up-down)
            double StrafeDistance_3D = fiducial.getRobotPoseTargetSpace().getOrientation().getYaw();

            telemetry.addData("Fiducial " + id, "is at" + x + y + StrafeDistance_3D);
        }

        List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
        for (LLResultTypes.DetectorResult detectionresult : detections) {
            String className = detectionresult.getClassName(); // What was detected
            double x = detectionresult.getTargetXDegrees(); // Where it is (left-right)
            double y = detectionresult.getTargetYDegrees(); // Where it is (up-down)
            telemetry.addData(className, "at (" + x + ", " + y + ") degrees");
        }

        long staleness = result.getStaleness();
        if (staleness < 100) { // Less than 100 milliseconds old
            telemetry.addData("Data", "Good");
        } else {
            telemetry.addData("Data", "Old (" + staleness + " ms)");
        }

        LLFieldMap fieldMap = new LLFieldMap(); // You'll need to fill this with field data
        boolean success = limelight.uploadFieldmap(fieldMap, null); // null means use the default slot
        if (success) {
            telemetry.addData("Field Map", "Uploaded successfully!");
        } else {
            telemetry.addData("Field Map", "Oops, upload failed");
        }

        limelight.captureSnapshot("auto_pov_10s");

        limelight.deleteSnapshots();
        telemetry.addData("Snapshots", "All cleared out!");
    }
}
