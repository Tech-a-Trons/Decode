package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems;

import com.acmerobotics.dashboard.config.Config;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;

//For new intake

@Config
public class TurretPID implements Subsystem {

    public static final TurretPID INSTANCE = new TurretPID();
    public static double kP = 0.05; //before 0.0005
    public static double kI = 0.0;
    public static double kD = 0.0;

    public static double kV = 0.002;
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
            .velPid(0.0005, 0, 0) // Velocity PID with kP=0.1, kI=0.01, kD=0.05
            .basicFF(0.0001, 0, 0) // Basic feedforward with kV=0.02, kA=0.0, kS=0.01
            .build();

    // Set the goal velocity to 500 units per second

    public Command setCloseShooterSpeed(){
        return new RunToVelocity(
                controller,
                900, // 500
                5
        ).requires(this);
    }

    public Command setFarShooterSpeed(){
        return new RunToVelocity(
                controller,
                1100, // 700
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
    public void periodic() {
        // Shooter velocity in ticks per second
        double vel = turret.getVelocity();

        // KineticState with ONLY velocity
        KineticState state = new KineticState(vel);

        // Compute controller output
        double output = controller.calculate(state);

        // Apply to both motors
        turret.setPower(output);
    }
}
