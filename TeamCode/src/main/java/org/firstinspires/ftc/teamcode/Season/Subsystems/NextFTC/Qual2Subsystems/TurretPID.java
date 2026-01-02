package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems.BlueExperimentalDistanceLExtractor;
import org.firstinspires.ftc.teamcode.Season.Subsystems.Sensors.PIDVoltageGet;
import org.firstinspires.ftc.teamcode.Season.Subsystems.Sensors.VoltageGet;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;

//For new intake

//@Config
public class TurretPID implements Subsystem {

    public static final TurretPID INSTANCE = new TurretPID();
    public static double kP = 0.0; //before 0.0005
    public static double kI = 0.0;
    public static double kD = 0.0;

    public static double kV = 0.0;
    public static double kA = 0.0;
    public static double kS = 0.01;
    public static double closegoal = 500;
    public static double fargoal = 700;
    public static boolean shootRequested = false;
    public static boolean hasShot = false;
    public static double VELO_TOL = 75;
    public static double activeTargetVelocity = 0;



    public static MotorGroup turret = new MotorGroup(
            new MotorEx("outtakeleft"),
            new MotorEx("outtakeright").reversed()
    );

    ControlSystem controller = ControlSystem.builder()
            .velPid(0.003, 0, 0) // Velocity PID with kP=0.0005, kI=0.01, kD=0.05
            .basicFF(0.0001, 0, 0.044) // Basic feedforward with kV=0.0001, kA=0.0, kS=0.01
            .build();

    // Set the goal velocity to 500 units per second



    public Command setCloseShooterSpeed(){
        //ll.update();
        //hood.INSTANCE.close();
        return new RunToVelocity(
                controller,
                1500, // distance*3.03 1500 close
                5
        ).requires(this);
    }
    public Command setShooterFromDistance(double distance) {
        double velocity = 0.041 * distance * distance - 2.9 * distance + 1350;
        velocity = Math.max(1200, Math.min(2000, velocity));
        activeTargetVelocity = velocity;
        shootRequested = true;
        hasShot = false;
        return new RunToVelocity(controller, velocity, 5).requires(this);
    }
    public Command newshooterdistance(double distance) {
        double velocity =
                0.041 * distance * distance
                        - 2.9 * distance
                        + 1350;

        if (distance > 60) {
            velocity -= 2.2 * (distance - 60);
        }

        velocity = Math.max(1200, Math.min(2000, velocity));

        activeTargetVelocity = velocity;
        shootRequested = true;
        hasShot = false;

        return new RunToVelocity(controller, velocity, 5).requires(this);
    }

    public Command setFarShooterSpeed(){
        //ll.update();
        //hood.INSTANCE.open();
        return new RunToVelocity(
                controller,
                1800, //1800 for far
                5
        ).requires(this);
    }
    public Command shotforyou(){
        //ll.update();
        //hood.INSTANCE.open();
        return new RunToVelocity(
                controller,
                1375, //1800 for far
                5
        ).requires(this);
    }

    public Command setMidCloseShooterSpeed(){
        //ll.update();
        //hood.INSTANCE.midclose();
        return new RunToVelocity(
                controller,
                1285, // 500
                5
        ).requires(this);
    }

    public Command setMidFarShooterSpeed(){
        //ll.update();
        //hood.INSTANCE.midopen();
        return new RunToVelocity(
                controller,
                975, // 700
                5
        ).requires(this);
    }

    public Command resetShooter(){
        //ll.update();
        return new RunToVelocity(
                controller,
                0,
                0
        ).requires(this);
    }

    @Override
    public void periodic(){
        turret.setPower(
                controller.calculate(
                        turret.getState()
                )
        );
    }
}