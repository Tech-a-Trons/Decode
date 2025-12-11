//package org.firstinspires.ftc.teamcode.Season.TeleOp.NewPrototypeTeleops;
//
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual1Subsystem.Intake;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual1Subsystem.Midtake;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual1Subsystem.Outtake;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.TurretOuttake;
//a
//import dev.nextftc.core.commands.Command;
//import dev.nextftc.core.components.BindingsComponent;
//import dev.nextftc.core.components.SubsystemComponent;
//import dev.nextftc.ftc.Gamepads;
//import dev.nextftc.ftc.NextFTCOpMode;
//import dev.nextftc.ftc.components.BulkReadComponent;
//import dev.nextftc.hardware.driving.MecanumDriverControlled;
//import dev.nextftc.hardware.impl.MotorEx;
//
//@TeleOp(name = "OuttakePID")
//
//public class OuttakePIDtest extends NextFTCOpMode {
//    public OuttakePIDtest() {
//        addComponents(
//                new SubsystemComponent(TurretOuttake.INSTANCE),
//                BulkReadComponent.INSTANCE,
//                BindingsComponent.INSTANCE
//        );
//    }
//
//    // change the names and directions to suit your robot
////    private final MotorEx frontLeftMotor = new MotorEx("fl").reversed();
////    private final MotorEx frontRightMotor = new MotorEx("fr");
////    private final MotorEx backLeftMotor = new MotorEx("bl").reversed();
////    private final MotorEx backRightMotor = new MotorEx("br");
//    private boolean intakeOn = false;
//
//    @Override
//    public void onStartButtonPressed() {
////        Command driverControlled = new MecanumDriverControlled(
////                frontLeftMotor,
////                frontRightMotor,
////                backLeftMotor,
////                backRightMotor,
////                Gamepads.gamepad1().leftStickY().negate(),
////                Gamepads.gamepad1().leftStickX(),
////                Gamepads.gamepad1().rightStickX()
////        );
//
//// Toggle all off when left bumper is pressed
//        Gamepads.gamepad1().leftBumper()
//                .whenBecomesTrue(() -> {
//                    TurretOuttake.INSTANCE.resetShooter().schedule();
//                    ;// stop outtake
////                    Midtake.INSTANCE.newtake.setPower(0);    // stop ramp/midtake
////                    Intake.INSTANCE.activeintake.setPower(0);     // stop intake
//                });
//        Gamepads.gamepad1().b().whenBecomesTrue(
//                () -> TurretOuttake.INSTANCE.setfarShooterSpeed().schedule()
//        );
//        Gamepads.gamepad1().rightBumper()
//                .whenBecomesTrue(() -> {
//                    // Start shooter immediately
//                    TurretOuttake.INSTANCE.setcloseShooterSpeed().schedule();
//
////                    // Schedule intake + midtake after 400 ms
////                    Intake.INSTANCE.activeintake.setPower(1);
////                    Midtake.INSTANCE.newtake.setPower(-1);
//                });
//
////        Gamepads.gamepad1().dpadUp()
////                .whenTrue(() -> Intake.INSTANCE.activeintake.setPower(1.0))   // run intake while held
////                .whenFalse(() -> Intake.INSTANCE.activeintake.setPower(0.0)); // stop intake when released
//
//        // Inside your periodic loop or command scheduler
//
//
////
////        driverControlled.schedule();
//    }
//}