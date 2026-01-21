package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.PedroCoordinates;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.Turret;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;


public class TurretOdo implements Subsystem {

    public static final TurretOdo INSTANCE = new TurretOdo();
    double x = 0;

    double y = 0;

    double heading = 0;

    // Black dot (origin of ang
    public void periodic() {
        Pose currentPose = PedroComponent.follower().getPose();
         x = currentPose.getX();
         y = currentPose.getY();
         heading = currentPose.getHeading();
    }
    // Black dot (origin of angle)
    double xb = x;
    double yb = y;

    // Red dot (target)
    double xr = 121;
    double yr = 121;

    double TurretAngle = 0;
    //Math Functions
    double yxangleDeg = Math.toDegrees(
            Math.atan2(yr - yb, xr - xb)
    );

    public void TurretAngleCalc() {
        TurretAngle = yxangleDeg - heading;
        if (TurretAngle < 0) {
            TurretAngle += 360;
        }

        // Servo Rotation

    };

    public double getx() {
        return x;
    }
    public double gety() {
        return y;
    }
    public double getheading() {
        return heading;
    }
    public double getTAngle() {
        return TurretAngle;
    }

}

