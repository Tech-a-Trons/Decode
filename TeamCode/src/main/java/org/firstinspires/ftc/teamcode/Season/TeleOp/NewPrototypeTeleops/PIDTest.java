package org.firstinspires.ftc.teamcode.Season.TeleOp.NewPrototypeTeleops;

import static org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.TurretPID.turret;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.TurretPID;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp
public class PIDTest extends NextFTCOpMode {
    public PIDTest() {
        addComponents(
                new SubsystemComponent(TurretPID.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    // change the names and directions to suit your robot
//    private final MotorEx frontLeftMotor = new MotorEx("fl").reversed();
//    private final MotorEx frontRightMotor = new MotorEx("fr");
//    private final MotorEx backLeftMotor = new MotorEx("bl").reversed();
//    private final MotorEx backRightMotor = new MotorEx("br");
//    private boolean intakeOn = false;
//
//   private final MotorEx outl = new MotorEx("outtakeleft").reversed();
//   private final MotorEx outr = new MotorEx("outtakeright");

    @Override
    public void onStartButtonPressed() {
        telemetry.addData("Velo: ", turret.getVelocity());
        telemetry.update();
//        Command driverControlled = new MecanumDriverControlled(
//                frontLeftMotor,
//                frontRightMotor,
//                backLeftMotor,
//                backRightMotor,
//                Gamepads.gamepad1().leftStickY().negate(),
//                Gamepads.gamepad1().leftStickX(),
//                Gamepads.gamepad1().rightStickX()
//        );
        Gamepads.gamepad1().b()
                .whenBecomesTrue(() -> {
                    TurretPID.INSTANCE.setCloseShooterSpeed().schedule();
                });
        Gamepads.gamepad1().a()
                .whenBecomesTrue(() -> {
                    TurretPID.INSTANCE.setFarShooterSpeed().schedule();
                });
        Gamepads.gamepad1().x()
                .whenBecomesTrue(() -> {
                    TurretPID.INSTANCE.resetShooter().schedule();
                });
        Gamepads.gamepad1().y()
                .whenBecomesTrue(() -> {
                   TurretPID.INSTANCE.periodic();
                });
        Gamepads.gamepad1().leftBumper()
                .whenBecomesTrue(() -> {
                    TurretPID.INSTANCE.turret.setPower(0);
                });
// Toggle all off when left bumper is pressed
//        Gamepads.gamepad1().leftBumper()
//                .whenBecomesTrue(() -> {
//                    Outtake.INSTANCE.resetShooter().schedule();
//                    ;// stop outtake
//                    Midtake.INSTANCE.newtake.setPower(0);    // stop ramp/midtake
//                    Intake.INSTANCE.activeintake.setPower(0);     // stop intake
//                });
//        Gamepads.gamepad1().b().whenBecomesTrue(
//                () -> Outtake.INSTANCE.setShooterSpeed().schedule()
//        );
//        Gamepads.gamepad1().rightBumper()
//                .whenBecomesTrue(() -> {
//                    // Start shooter immediately
//                    Outtake.INSTANCE.setShooterSpeed().schedule();
//
//                    // Schedule intake + midtake after 400 ms
//                    Intake.INSTANCE.activeintake.setPower(1);
//                    Midtake.INSTANCE.newtake.setPower(-1);
//                });
//        Gamepads.gamepad1().a().whenBecomesTrue(() -> {
//            intakeOn = !intakeOn; // flip state
//            Intake.INSTANCE.activeintake.setPower(intakeOn ? 1.0 : 0.0); // set power based on toggle
//        });
//        Gamepads.gamepad1().dpadUp()
//                .whenTrue(() -> Intake.INSTANCE.activeintake.setPower(1.0))   // run intake while held
//                .whenFalse(() -> Intake.INSTANCE.activeintake.setPower(0.0)); // stop intake when released

        // Inside your periodic loop or command scheduler



        //driverControlled.schedule();
    }
}
