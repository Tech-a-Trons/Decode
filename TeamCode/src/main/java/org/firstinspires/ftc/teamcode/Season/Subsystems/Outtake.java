package org.firstinspires.ftc.teamcode.Season.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.firstinspires.ftc.teamcode.Season.Pedro.Constants;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;

public class Outtake implements Subsystem {
    public static final Outtake INSTANCE = new Outtake();
    private Outtake() { }
    public static MotorGroup outtake = new MotorGroup(
            new MotorEx("outtake1").reversed(),
            new MotorEx("outtake2")
    );

    ControlSystem controller = ControlSystem.builder()
            .velPid(0.1, 0.01, 0.05) // Velocity PID with kP=0.1, kI=0.01, kD=0.05
            .basicFF(0.02, 0.0, 0.01) // Basic feedforward with kV=0.02, kA=0.0, kS=0.01
            .build();

    // Set the goal velocity to 500 units per second

    public Command setShooterSpeed(){

        return new RunToVelocity(
                controller,
                600,
                5
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

