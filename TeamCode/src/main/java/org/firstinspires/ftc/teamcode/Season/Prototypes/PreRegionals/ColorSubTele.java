//package org.firstinspires.ftc.teamcode.Season.Prototypes.PreRegionals;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.ColorSensor;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.CompliantIntake;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.Transfer;
//
//import dev.nextftc.core.components.SubsystemComponent;
//import dev.nextftc.ftc.NextFTCOpMode;
//
//@Disabled
//@TeleOp(name = "ColorSub")
//public class ColorSubTele extends NextFTCOpMode {
//
//    private ColorSensor colorSensor;
//    private CompliantIntake compliantIntake;
//
//    private Transfer transfer;
//
//    public ColorSubTele() {
//        // Register subsystems if they extend SubsystemBase / use INSTANCE pattern
//        addComponents(
//                new SubsystemComponent(
//                        CompliantIntake.INSTANCE
//                )
//        );
//    }
//
//    @Override
//    public void onStartButtonPressed() {
//        // ✅ Initialize hardware here
//        colorSensor = new ColorSensor();
//        compliantIntake = CompliantIntake.INSTANCE;
//        transfer = transfer.INSTANCE;
//
//
//        telemetry.update();
//    }
//
//    @Override
//    public void onUpdate() {
//        telemetry.addData("Artifact Count", colorSensor.artifactcounter);
////        telemetry.addData("Green", colorSensor.green);
////        telemetry.addData("Purple", colorSensor.purple);
//        telemetry.addData("Hue", colorSensor.current_hue);
//        telemetry.addData("Sat", colorSensor.current_sat);
//        telemetry.addData("Value", colorSensor.current_val);
//        // ✅ Controls
//        if (gamepad1.a) {
//            compliantIntake.on();
//            transfer.slight();
//            colorSensor.IncountBalls();
//
//        } else {
//            compliantIntake.off();
//            transfer.off();
//        }
//        telemetry.update();
//    }
//}
