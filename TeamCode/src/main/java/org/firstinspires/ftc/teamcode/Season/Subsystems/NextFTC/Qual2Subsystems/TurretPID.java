package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems;

import com.acmerobotics.dashboard.config.Config;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;

//For new intake

@Config
public class TurretPID implements Subsystem {

    public static final TurretPID INSTANCE = new TurretPID();
    public static double kP = 0.001;
    public static double kI = 0.01;
    public static double kD = 0.05;

    public static double kV = 0.02;
    public static double kA = 0.0;
    public static double kS = 0.01;
    public static double closegoal = 500;
    public static double fargoal = 700;

    private TurretPID() { }

    public static MotorGroup turret = new MotorGroup(
            new MotorEx("outtakeright"),
            new MotorEx("outtakeleft").reversed()
    );

    ControlSystem controller = ControlSystem.builder()
            .velPid(0.0000005, 0.0, 0.0) // Velocity PID with kP=0.1, kI=0.01, kD=0.05
            .basicFF(0.02, 0.0, 0.01) // Basic feedforward with kV=0.02, kA=0.0, kS=0.01
            .build();

    // Set the goal velocity to 500 units per second

    public Command setCloseShooterSpeed(){

        return new RunToVelocity(
                controller,
                0.5,
                5
        ).requires(this);
    }

    public Command setFarShooterSpeed(){

        return new RunToVelocity(
                controller,
                1,
                5
        ).requires(this);
    }
    public Command resetShooter(){

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
        turret.getVelocity();
    }
}
