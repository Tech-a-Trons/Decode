package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems.BlueExperimentalDistanceLExtractor;
import org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems.RedExperimentalDistanceLExtractor;
import org.firstinspires.ftc.teamcode.Season.Subsystems.Sensors.PIDVoltageGet;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.impl.MotorEx;

public class DistanceRedTurretPID implements Subsystem {
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
    RedExperimentalDistanceLExtractor ll;

    /* ---------------- Constructor ---------------- */
    public DistanceRedTurretPID(HardwareMap hardwareMap) {

        turret = new MotorGroup(
                new MotorEx("outtakeright"),
                new MotorEx("outtakeleft").reversed()
        );

        ll = new RedExperimentalDistanceLExtractor(hardwareMap);
        ll.startReading();

        VoltageSensor sensor = hardwareMap.voltageSensor.iterator().next();
        volt = new PIDVoltageGet(sensor);

        controller = ControlSystem.builder()
                .velPid(0.0006, 0, 0)
                .basicFF(0.0001, 0, 0.044)
                .build();
    }

    /* ---------------- Public API ---------------- */
    public void setShooterSpeed() {
        ll.startReading();
        ll.update();
        double distance = ll.getEuclideanDistance();
        targetVelocity = distance*3.03;
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
