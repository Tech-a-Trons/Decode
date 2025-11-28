//package org.firstinspires.ftc.teamcode.Personal.Niranjan.MercurialV1BUGGED;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import org.firstinspires.ftc.teamcode.Season.Subsystems.Mercurial.JavaSubsystem;
//
//import dev.frozenmilk.dairy.pasteurized.SDKGamepad;
//import dev.frozenmilk.mercurial.Mercurial;
//import dev.frozenmilk.mercurial.bindings.BoundGamepad;
//import dev.frozenmilk.mercurial.commands.Lambda;
//import dev.frozenmilk.mercurial.commands.groups.Parallel;
//import dev.frozenmilk.mercurial.commands.groups.Sequential;
//import dev.frozenmilk.mercurial.commands.util.Wait;
//
//@Mercurial.Attach
//@JavaSubsystem.Attach
//@DriveSubsystem.Attach
//@IntakeSubsystem.Attach
//@OuttakeSubsystem.Attach
//@RampSubsystem.Attach
//public class MercurialTestTeleop extends OpMode {
//
//    private boolean intakeState = false; // false = off, true = on
//
//    @Override
//    public void init() {
//        OuttakeSubsystem outtake = OuttakeSubsystem.INSTANCE;
//        RampSubsystem ramp = RampSubsystem.INSTANCE;
//        IntakeSubsystem intake = IntakeSubsystem.INSTANCE;
//        BoundGamepad driver = new BoundGamepad(new SDKGamepad(gamepad1));
//
//        // Right bumper = shoot sequence
//        driver.rightBumper().onTrue(
//                new Sequential(
//                        // Spin outtake motors using the existing default command
//                        OuttakeSubsystem.runOuttakeCommand(),
//
//                        // Wait for 0.65 seconds
//                        new Wait(0.65),
//
//                        // Start ramp & intake in parallel
//                        new Parallel(
//                                new Lambda("Ramp Start").setInit(() -> RampSubsystem.INSTANCE.setPower(-1)),
//                                new Lambda("Intake Start").setInit(() -> IntakeSubsystem.INSTANCE.in())
//                        )
//                )
//        );
//        // Left bumper = stop all subsystems except drive
//        driver.leftBumper().onTrue(
//                new Lambda("Stop All Except Drive").setInit(() -> {
//                    // Stop outtake motors
//                    OuttakeSubsystem.INSTANCE.stop();
//
//                    // Stop ramp motor
//                    RampSubsystem.INSTANCE.stop();
//
//                    // Stop intake motor
//                    IntakeSubsystem.INSTANCE.stop();
//                })
//        );
//
//        // Button A = Intake toggle
//        driver.a().onTrue(
//                new Lambda("Toggle Intake").setInit(() -> {
//                    intakeState = !intakeState;
//                    if(intakeState) intake.in();
//                    else intake.stop();
//                })
//        );
//    }
//
//    @Override
//    public void loop() {
//        double drive = -gamepad1.left_stick_y;
//        double strafe = gamepad1.left_stick_x;
//        double turn = gamepad1.right_stick_x;
//
//        // Drive
//        DriveSubsystem.joystickDriveCommand().execute();
//
//        telemetry.addData("Intake", intakeState ? "ON" : "OFF");
//        telemetry.update();
//    }}