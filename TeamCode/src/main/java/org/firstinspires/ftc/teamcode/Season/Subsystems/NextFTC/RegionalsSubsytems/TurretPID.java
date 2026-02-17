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

    public Command regionalsshooterdistance(double distance) {
        newvelo =
                0.041 * distance * distance
                        - 2.9 * distance
                        + 1400; //1350

        if (distance > 60) {
            newvelo -= 3 * (distance - 60);
        }

        newvelo= Math.max(1200, Math.min(4000, newvelo));

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
            double vx, double vy,
            double goalX, double goalY) {

        // ========== 1. COMPUTE DISTANCE TO GOAL ==========
        double dx = goalX - robotX;
        double dy = goalY - robotY;
        double distance = Math.hypot(dx, dy);

        // ========== 2. COMPUTE BASE FLYWHEEL VELOCITY ==========
        double flywheelVelo = 0.041 * distance * distance
                - 2.9 * distance
                + 1050;

        if (distance > 60) {
            flywheelVelo -= 2.2 * (distance - 60);
        }

        // ========== 3. COMPENSATE FOR ROBOT VELOCITY ==========
        double shotDirX = dx / distance;
        double shotDirY = dy / distance;
        double velocityTowardGoal = vx * shotDirX + vy * shotDirY;

        // ONLY compensate if moving significantly (avoid noise when stationary)
        double VELOCITY_THRESHOLD = 2.0;  // inches/sec
        double compensation = 0;

        if (Math.abs(velocityTowardGoal) > VELOCITY_THRESHOLD) {
            // Use quadratic scaling but with safety limits
            double baseGain = 0.002;  // REDUCED - start smaller
            double velocityCompensationGain = baseGain * distance * distance;

            // Calculate compensation
            compensation = velocityTowardGoal * velocityCompensationGain;

            // LIMIT compensation to prevent going negative or too high
            double MAX_COMPENSATION = 300;  // Don't adjust by more than 300
            compensation = Math.max(-MAX_COMPENSATION, Math.min(MAX_COMPENSATION, compensation));
        }

        flywheelVelo += compensation;

        // Clamp to safe operational range
        flywheelVelo = Math.max(900, Math.min(2000, flywheelVelo));

        // ========== 4. SET SHOOTER STATE FLAGS ==========
        newactv = flywheelVelo;
        shootRequested = true;
        hasShot = false;
        activeTargetVelocity = flywheelVelo;

        // ========== 5. RETURN COMMAND TO SPIN UP FLYWHEEL ==========
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