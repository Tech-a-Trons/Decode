package org.firstinspires.ftc.teamcode.Season.Prototypes.Regionals;

//import static org.firstinspires.ftc.teamcode.Season.Auto.Tuning.telemetryM;

import android.health.connect.TimeRangeFilter;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems.OdoTrackRed;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual1Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.CompliantIntake;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.Transfer;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.TurretOdoAi;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.TurretPID;

import dev.nextftc.core.commands.Command;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp(name = "Tuff Shooting")
public class PleaseSpeedINeedThis extends OpMode {

    private long intakeStartTime;
    private boolean intakeToggle = false;
    TurretPID turretPID;
    TurretOdoAi turret;
    CompliantIntake intake;
    Transfer transfer;
    //Drive
    private final MotorEx fl = new MotorEx("fl").reversed();
    private final MotorEx fr = new MotorEx("fr");
    private final MotorEx bl = new MotorEx("bl").reversed();
    private final MotorEx br = new MotorEx("br");

   // ===== Velocity tracking =====
    private double lastX, lastY;
    private double lastTime;

    @Override
    public void init() {


        lastTime = System.nanoTime() / 1e9;
        lastX = turret.getX();
        lastY = turret.getY();
        intake = new CompliantIntake();
        transfer = new Transfer();
    }

    @Override
    public void loop() {
        // ===== Update odometry =====
        double currentTime = System.nanoTime() / 1e9;
        double dt = currentTime - lastTime;
        lastTime = currentTime;

        double robotX = turret.getX();
        double robotY = turret.getY();
        double robotHeadingDeg = turret.getHeading();
        double robotHeadingRad = Math.toRadians(robotHeadingDeg);

        // ===== Compute field velocity (inches/sec) =====
        double robotVx = (robotX - lastX) / dt;
        double robotVy = (robotY - lastY) / dt;

        lastX = robotX;
        lastY = robotY;

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 0.75);

        double flPower = (y+x+rx) / denominator;
        double frPower = (y-x-rx) / denominator;
        double blPower = (y-x+rx) / denominator;
        double brPower = (y+x-rx) / denominator;

        fl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        fl.setPower(flPower);
        fr.setPower(frPower);
        bl.setPower(blPower);
        br.setPower(brPower);

        // Keep a reference to the active shooting command

        boolean isshot = false;

        Gamepads.gamepad1().rightBumper()
                .whenBecomesTrue(() -> {
                    intakeToggle = !intakeToggle;

                    if (intakeToggle) {
                        // Starting intake
                        intakeStartTime = System.currentTimeMillis();
                        CompliantIntake.INSTANCE.on();
                        Transfer.INSTANCE.slight();

                        // Reset ball counter when starting intake
                       // colorSensor.artifactcounter = 0;

                        //telemetry.addData("Intake", "ON - Counting Balls");
                    } else {
                        // Manually stopping intake
                        intakeStartTime = 0;
                        CompliantIntake.INSTANCE.off();
                        Transfer.INSTANCE.off();

                        // Reset ball counter
//                        colorSensor.artifactcounter = 0;

                        //telemetry.addData("Intake", "OFF - Manual Stop");
                    }
                });

        if (gamepad1.a && !isshot) {

            y = -gamepad1.left_stick_y;
            x = gamepad1.left_stick_x * 1.1;
            rx = gamepad1.right_stick_x;
            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 0.75);

            flPower = (y+x+rx) / denominator;
            frPower = (y-x-rx) / denominator;
            blPower = (y-x+rx) / denominator;
            brPower = (y+x-rx) / denominator;

            fl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            bl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            fr.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            br.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

            fl.setPower(flPower);
            fr.setPower(frPower);
            bl.setPower(blPower);
            br.setPower(brPower);

            turret.periodic();
            Command activeShootCommand = turretPID.tuffshot(
                    robotX, robotY, robotHeadingRad,
                    robotVx, robotVy,
                    121.0, 121.0   // goal
            );
            activeShootCommand.run();
            isshot = true;
        }

        if (gamepad1.a && isshot) {

            y = -gamepad1.left_stick_y;
            x = gamepad1.left_stick_x * 1.1;
            rx = gamepad1.right_stick_x;
            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 0.75);

            flPower = (y+x+rx) / denominator;
            frPower = (y-x-rx) / denominator;
            blPower = (y-x+rx) / denominator;
            brPower = (y+x-rx) / denominator;

            fl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            bl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            fr.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            br.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

            fl.setPower(flPower);
            fr.setPower(frPower);
            bl.setPower(blPower);
            br.setPower(brPower);

            turretPID.turret.setPower(0);

            isshot = false;

            // reset any TurretPID flags if needed
            turretPID.hasShot = false;
            turretPID.shootRequested = false;
        }

        telemetry.addLine("===== ODOMETRY =====");
        telemetry.addData("Pose", "%.1f, %.1f @ %.1f°",
                robotX, robotY, robotHeadingDeg);
        telemetry.addData("Velocity (in/s)", "Vx: %.2f  Vy: %.2f",
                robotVx, robotVy);

        telemetry.addLine("===== MOVING AIM =====");
        telemetry.addData("Distance", "%.1f in", turret.getDistanceToTarget());

        telemetry.update();
    }
}