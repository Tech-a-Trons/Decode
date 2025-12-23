package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual1Subsystem;


import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import com.acmerobotics.dashboard.config.Config;
@Config

//Old outtake Subsystem
public class Outtake implements Subsystem {

    public static final Outtake INSTANCE = new Outtake();
    public static double kP = 0.00005;
    public static double kI = 0.01;
    public static double kD = 0.05;

    public static double kV = 0.02;
    public static double kA = 0.0;
    public static double kS = 0.01;

    public static double goal = 700;
    private Outtake() { }

    public static MotorGroup outtake = new MotorGroup(
            new MotorEx("outtakeleft"),
            new MotorEx("outtakeright").reversed()
    );

    ControlSystem controller = ControlSystem.builder()
            .velPid(0.0006, 0, 0) // Velocity PID with kP=0.1, kI=0.01, kD=0.05
            .basicFF(0.0001, 0.0, 0.044) // Basic feedforward with kV=0.02, kA=0.0, kS=0.01
            .build();

    // Set the goal velocity to 500 units per second

    public Command setShooterSpeed(){

        return new RunToVelocity(
                controller,
                825,
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
