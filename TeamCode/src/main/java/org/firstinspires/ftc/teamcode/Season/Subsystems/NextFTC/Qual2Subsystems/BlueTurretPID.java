//package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems;
//
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems.BlueExperimentalDistanceLExtractor;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.Sensors.PIDVoltageGet;
//import org.firstinspires.ftc.teamcode.Season.Subsystems.Sensors.VoltageGet;
//
//import dev.nextftc.control.ControlSystem;
//import dev.nextftc.control.KineticState;
//import dev.nextftc.core.commands.Command;
//import dev.nextftc.core.subsystems.Subsystem;
//import dev.nextftc.ftc.NextFTCOpMode;
//import dev.nextftc.hardware.controllable.MotorGroup;
//import dev.nextftc.hardware.controllable.RunToVelocity;
//import dev.nextftc.hardware.impl.MotorEx;
//
////For new intake
//
//@Config
//public class BlueTurretPID implements Subsystem {
//
//    public static final BlueTurretPID INSTANCE = new BlueTurretPID();
//    public static double kP = 0.05; //before 0.0005
//    public static double kI = 0.0;
//    public static double kD = 0.0;
//
//    public static double kV = 0.002;
//    public static double kA = 0.0;
//    public static double kS = 0.01;
//    public static double closegoal = 500;
//    public static double fargoal = 700;
//    PIDVoltageGet volt = new PIDVoltageGet(
//            hardwareMap.voltageSensor.iterator().next()
//    );
//
//    Hood hood = new Hood();
////    BlueExperimentalDistanceLExtractor ll = new BlueExperimentalDistanceLExtractor();
////    double distance = ll.getEuclideanDistance();
//
//    public BlueTurretPID() {
//        //ll.startReading();
//    }
//
//    public static MotorGroup turret = new MotorGroup(
//            new MotorEx("outtakeright"),
//            new MotorEx("outtakeleft").reversed()
//    );
//
//    ControlSystem controller = ControlSystem.builder()
//            .velPid(0.0006, 0, 0) // Velocity PID with kP=0.0005, kI=0.01, kD=0.05
//            .basicFF(0.0001, 0, 0.044) // Basic feedforward with kV=0.0001, kA=0.0, kS=0.01
//            .build();
//
//    // Set the goal velocity to 500 units per second
//
//    public Command motor1 () {
//        turret.setPower(1);
//        return new Command() {
//            @Override
//            public boolean isDone() {
//                return false;
//            }
//        };
//    }
//
//    public Command setCloseShooterSpeed(){
//        //ll.update();
//        //hood.INSTANCE.close();
//        return new RunToVelocity(
//                controller,
//                825, // distance*3.03
//                5
//        ).requires(this);
//    }
//
//    public Command setFarShooterSpeed(){
//        //ll.update();
//        //hood.INSTANCE.open();
//        return new RunToVelocity(
//                controller,
//                1025, // 700
//                5
//        ).requires(this);
//    }
//
//    public Command setMidCloseShooterSpeed(){
//        //ll.update();
//        //hood.INSTANCE.midclose();
//        return new RunToVelocity(
//                controller,
//                900, // 500
//                5
//        ).requires(this);
//    }
//
//    public Command setMidFarShooterSpeed(){
//        //ll.update();
//        //hood.INSTANCE.midopen();
//        return new RunToVelocity(
//                controller,
//                975, // 700
//                5
//        ).requires(this);
//    }
//
//    public Command resetShooter(){
//        //ll.update();
//        return new RunToVelocity(
//                controller,
//                0,
//                0
//        ).requires(this);
//    }
//
//    @Override
//    public void periodic() {
//
//    }
//}

//Need to test/tune this new code...
package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Season.Subsystems.Sensors.PIDVoltageGet;
import org.firstinspires.ftc.teamcode.Season.Subsystems.Sensors.VoltageGet;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.impl.MotorEx;

public class BlueTurretPID implements Subsystem {

    /* ---------------- Tunables ---------------- */
    public static double kP = 0.0006;
    public static double kI = 0.0;
    public static double kD = 0.0;

    public static double kV = 0.0001;
    public static double kA = 0.0;
    public static double kS = 0.044;

    /* ---------------- Hardware ---------------- */
    private final MotorGroup turret;
    private final PIDVoltageGet volt;

    /* ---------------- Control ---------------- */
    private final ControlSystem controller;
    private double targetVelocity = 0.0;

    /* ---------------- Constructor ---------------- */
    public BlueTurretPID(HardwareMap hardwareMap) {

        turret = new MotorGroup(
                new MotorEx("outtakeright"),
                new MotorEx("outtakeleft").reversed()
        );

        VoltageSensor sensor = hardwareMap.voltageSensor.iterator().next();
        volt = new PIDVoltageGet(sensor);

        controller = ControlSystem.builder()
                .velPid(0.0006, 0, 0)
                .basicFF(0.0001, 0, 0.044)
                .build();
    }

    /* ---------------- Public API ---------------- */
    public void setCloseShooterSpeed() {
        targetVelocity = 825;
    }

    public void setMidCloseShooterSpeed() {
        targetVelocity = 900;
    }

    public void setMidFarShooterSpeed() {
        targetVelocity = 975;
    }

    public void setFarShooterSpeed() {
        targetVelocity = 1025;
    }

    public void stopShooter() {
        targetVelocity = 0.0;
    }

    public double getVelocity() {
        return turret.getVelocity();
    }

    /* ---------------- Control Loop ---------------- */
    @Override
    public void periodic() {
        double currentVelocity = turret.getVelocity();

        // measured, target
        KineticState state = new KineticState(
                currentVelocity,
                targetVelocity
        );

        telemetry.addData("Shooter Velocity", currentVelocity);
        telemetry.addData("Target", targetVelocity);
        telemetry.update();

        double output = controller.calculate(state);

        double compensated = volt.regulate(output);

        turret.setPower(
                Math.max(-1.0, Math.min(1.0, compensated))
        );
    }
}
