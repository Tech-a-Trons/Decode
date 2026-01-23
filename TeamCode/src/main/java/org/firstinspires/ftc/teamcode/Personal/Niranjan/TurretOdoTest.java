package org.firstinspires.ftc.teamcode.Personal.Niranjan;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
@Disabled
@TeleOp(name="TurretOdoTest")
public class TurretOdoTest extends LinearOpMode {

    DcMotor fr, fl, br, bl;
    CRServo turretServo;

    double robotX = 0;
    double robotY = 0;
    double turretCurrentAngle = 0;

    double lookahead = 5.0;
    double targetX = 0;
    double targetY = 0;

    @Override
    public void runOpMode() {

        fr = hardwareMap.get(DcMotor.class, "fr");
        fl = hardwareMap.get(DcMotor.class, "fl");
        br = hardwareMap.get(DcMotor.class, "br");
        bl = hardwareMap.get(DcMotor.class, "bl");
        turretServo = hardwareMap.get(CRServo.class, "turretServo");

        waitForStart();

        while (opModeIsActive()) {

            // --- Pedro Pathing for robot movement ---
            double[] pathTarget = getPedroTarget(robotX, robotY, lookahead);
            targetX = pathTarget[0];
            targetY = pathTarget[1];

            double[] pedroOutput = updatePedro(robotX, robotY);
            robotX = pedroOutput[0];
            robotY = pedroOutput[1];

            // --- Turret tracking ---
            double dx = targetX - robotX;
            double dy = targetY - robotY;
            double targetAngle = Math.toDegrees(Math.atan2(dy, dx));

            double error = targetAngle - turretCurrentAngle;

            if (Math.abs(error) < 2) {
                turretServo.setPower(0);
            } else {
                double power = Math.max(-1, Math.min(1, error / 30.0));
                turretServo.setPower(power);
            }

            turretCurrentAngle = getTurretAngle();

            // --- Gamepad controls for manual override (optional) ---
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            fr.setPower(drive - strafe - turn);
            fl.setPower(drive + strafe + turn);
            br.setPower(drive + strafe - turn);
            bl.setPower(drive - strafe + turn);
        }
    }

    // Stub for Pedro Pathing: returns next lookahead point
    private double[] getPedroTarget(double x, double y, double lookahead) {
        // replace with actual pathing logic
        return new double[]{x + 1, y};
    }

    // Stub for Pedro Pathing: updates robot position
    private double[] updatePedro(double x, double y) {
        // replace with actual pathing logic
        return new double[]{x + 0.5, y};
    }

    // Stub to get current turret angle
    private double getTurretAngle() {
        // replace with encoder or odometry-based tracking
        return turretCurrentAngle;
    }
}