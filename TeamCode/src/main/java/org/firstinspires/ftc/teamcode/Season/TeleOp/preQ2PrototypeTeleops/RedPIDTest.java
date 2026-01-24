package org.firstinspires.ftc.teamcode.Season.TeleOp.preQ2PrototypeTeleops;

//import static org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.BlueTurretPID.turret;
import static org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.ExperimentalRedTurretPID.turret;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems.RedExperimentalDistanceLExtractor;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.ExperimentalRedTurretPID;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.Hood;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Disabled
@TeleOp
public class RedPIDTest extends NextFTCOpMode {
    RedExperimentalDistanceLExtractor ll = new RedExperimentalDistanceLExtractor(hardwareMap);
    public RedPIDTest() {
        addComponents(
                new SubsystemComponent(ExperimentalRedTurretPID.INSTANCE, Hood.INSTANCE),
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
        final double STARGET_DISTANCE = 58; // inches 42.97
        final double SANGLE_TOLERANCE = -1.8;
        //    private final double MTARGET_DISTANCE = 2838; // PLACEHOLDER
//    private final double MANGLE_TOLERANCE = 134; // PLACEHOLDER
        final double FTARGET_DISTANCE = 124; //115
        final double FANGLE_TOLERANCE = 1; //3.47
        ll.startReading();
        ll.setTelemetry(telemetry);
        Double tx = ll.getTx();
        if (tx == null) {
            tx = 0.0;
        }
        Double distance = ll.getEuclideanDistance();
        if (distance == null) {
            distance = 0.0;
        }
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

        //Set the speed FIRST
        ll.update();
        Gamepads.gamepad1().b()
                .whenBecomesTrue(() -> {
                    ll.update();
                    ExperimentalRedTurretPID.INSTANCE.setCloseShooterSpeed().schedule();
//                    Hood.INSTANCE.open();

//                    TurretPID.INSTANCE.periodic();
                });
        Gamepads.gamepad1().a()
                .whenBecomesTrue(() -> {
                    ll.update();
                    ExperimentalRedTurretPID.INSTANCE.setFarShooterSpeed().schedule();
//                    Hood.INSTANCE.open();
//                    TurretPID.INSTANCE.periodic();
                });

        //Ehh might remove this
        Gamepads.gamepad1().x()
                .whenBecomesTrue(() -> {
                    ll.update();
                    ExperimentalRedTurretPID.INSTANCE.resetShooter().schedule();
//                    Hood.INSTANCE.close();
                });

        Gamepads.gamepad1().dpadLeft()
                .whenBecomesTrue(() -> {
                    ll.update();
                    ExperimentalRedTurretPID.INSTANCE.setMidCloseShooterSpeed().schedule();
//                    Hood.INSTANCE.open();
//                    TurretPID.INSTANCE.periodic();
                });

        Gamepads.gamepad1().dpadRight()
                .whenBecomesTrue(() -> {
                    ll.update();
                    ExperimentalRedTurretPID.INSTANCE.setMidFarShooterSpeed().schedule();
//                    Hood.INSTANCE.open();
//                    TurretPID.INSTANCE.periodic();
                });

        //Then set periodic
        Gamepads.gamepad1().y()
                .whenBecomesTrue(() -> {
                    ll.update();
                    ExperimentalRedTurretPID.INSTANCE.periodic();
                    ExperimentalRedTurretPID.INSTANCE.turret.setPower(0.6);
                });

        //If anything goes wrong, use this
        Gamepads.gamepad1().leftBumper()
                .whenBecomesTrue(() -> {
                    ll.update();
                    ExperimentalRedTurretPID.INSTANCE.turret.setPower(0);
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
