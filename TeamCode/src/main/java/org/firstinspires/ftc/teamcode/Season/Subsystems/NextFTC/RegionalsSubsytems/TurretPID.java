package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
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

    double newvelo;

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

    public ControlSystem controller = ControlSystem.builder()
            .velPid(0.003, 0, 0) // Velocity PID with 0.003
            .basicFF(0.0001, 0, 0.044) // Basic feedforward with kV=0.0001, kA=0.0, kS=0.01
            .build();

    public Command setShooterSpeed(double targetVelocity) {
        return new RunToVelocity(
                controller,
                targetVelocity,
                5
        ).requires(this);
    }
    // Set the goal velocity to 500 units per second


    public Command setCloseShooterSpeed(){
        //ll.update();
        //hood.INSTANCE.close();
        return new RunToVelocity(
                controller,
                1400, // distance*3.03 1500 close
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

    double newactv;
    public Command newshooterdistance(double distance) {
         newvelo =
                0.041 * distance * distance
                        - 2.9 * distance
                        + 1400; //1350

        if (distance > 60) {
            newvelo -= 2.2 * (distance - 60);
        }

        newvelo= Math.max(1200, Math.min(2000, newvelo));

        newactv = newvelo;
        shootRequested = true;
        hasShot = false;

        return new RunToVelocity(controller, newvelo, 5).requires(this);
    }

    public Command setFarShooterSpeed(){
        //ll.update();
        //hood.INSTANCE.open();
        return new RunToVelocity(
                controller,
                1600, //1800 for far
                5
        ).requires(this);
    }

    public Command tuffshot(
            double robotX, double robotY,
            double robotHeadingRad,
            double vx, double vy,
            double goalX, double goalY) {

        // --------- 1. Compute distance to goal ---------
        double dx = goalX - robotX;
        double dy = goalY - robotY;
        double distance = Math.hypot(dx, dy);

        // --------- 2. Compute flywheel velocity ---------
        double flywheelVelo = 0.041 * distance * distance
                - 2.9 * distance
                + 1350;
        if (distance > 60) flywheelVelo -= 2.2 * (distance - 60);
        flywheelVelo = Math.max(1200, Math.min(2000, flywheelVelo));

        // --------- 3. Store for lead calculations ---------
        newactv = flywheelVelo;

        // --------- 4. Optionally, compute turret angle for moving shot ---------
        double projectileSpeed = flywheelVelo * 0.08; // tune this to inches/sec
        double timeToTarget = (projectileSpeed > 0.001) ? distance / projectileSpeed : 0;

        double leadX = dx - vx * timeToTarget;
        double leadY = dy - vy * timeToTarget;
        double globalAngle = Math.atan2(leadY, leadX);
        double turretAngleRad = Math.atan2(
                Math.sin(globalAngle - robotHeadingRad),
                Math.cos(globalAngle - robotHeadingRad)
        );

        // --------- 5. Store turret angle in member variable if needed ---------
        double currentTurretTarget = turretAngleRad; // assuming you have a variable for PID

        // --------- 6. Return RunToVelocity command (sets shooter velocity) ---------
        return new RunToVelocity(controller, newactv, 5)
                .requires(this);
    }


//    private double computeDistance(double dx, double dy) {
//        return Math.hypot(dx, dy);
//    }
//
//    private double computeFlightTime(double distance, double projectileSpeed) {
//        if (projectileSpeed <= 0.001) return 0.0;
//        return distance / projectileSpeed;
//    }
//
//    private double computeLead(double delta, double robotVelocity, double time) {
//        return delta - robotVelocity * time;
//    }
//
//    private double estimateProjectileSpeed(double flywheelVelocity) {
//
//        double kVelocityToMetersPerSec = 0.0065; // TUNE THIS
//
//        return flywheelVelocity * kVelocityToMetersPerSec;
//    }
//
//    private double angleWrap(double angle) {
//        return Math.atan2(Math.sin(angle), Math.cos(angle));
//    }

    public Command shotforyou(){
        //ll.update();
        //hood.INSTANCE.open();
        return new RunToVelocity(
                controller,
                1265, //1800 for far
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
                1550, // 700
                5
        ).requires(this);
    }
    public Command tuffashell(){
        //ll.update();
        //hood.INSTANCE.midopen();
        return new RunToVelocity(
                controller,
                1570, // 700
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

    public double newgetActiveVelocity() {
        return newactv;
    }

    @Override
    public void periodic(){
        turret.setPower(
                controller.calculate(
                        turret.getState()
                ));


    }
}