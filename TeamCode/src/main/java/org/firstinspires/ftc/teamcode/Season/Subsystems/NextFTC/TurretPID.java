package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC;

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
    public static double kP = 0.00005;
    public static double kI = 0.01;
    public static double kD = 0.05;

    public static double kV = 0.02;
    public static double kA = 0.0;
    public static double kS = 0.01;
    public static double closegoal = 700;
    public static double fargoal = 1000;

    private TurretPID() { }

    public static MotorGroup outtake = new MotorGroup(
            new MotorEx("shooter1").reversed(),
            new MotorEx("shooter2")
    );

    ControlSystem controller = ControlSystem.builder()
            .velPid(kP, kI, kD) // Velocity PID with kP=0.1, kI=0.01, kD=0.05
            .basicFF(kV, kA, kS) // Basic feedforward with kV=0.02, kA=0.0, kS=0.01
            .build();

    // Set the goal velocity to 500 units per second

    public Command setCloseShooterSpeed(){

        return new RunToVelocity(
                controller,
                closegoal,
                5
        ).requires(this);
    }

    public Command setFarShooterSpeed(){

        return new RunToVelocity(
                controller,
                fargoal,
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
        outtake.setPower(
                controller.calculate(
                        outtake.getState()
                )
        );
    }
}
