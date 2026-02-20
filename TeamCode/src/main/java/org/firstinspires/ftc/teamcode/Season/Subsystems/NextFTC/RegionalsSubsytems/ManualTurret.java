package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.core.subsystems.Subsystem;


public class ManualTurret implements Subsystem {

    public static final ManualTurret INSTANCE = new ManualTurret();

    private Servo servo1;
    private Servo servo2;

    private ManualTurret() {}

    public void init(HardwareMap hardwareMap) {
        servo1 = hardwareMap.get(Servo.class, "turretServo1");
        servo2 = hardwareMap.get(Servo.class, "turretServo2");

        // If you ever need one reversed:
        // servo2.setDirection(Servo.Direction.REVERSE);
    }

    /**
     * Sets both servos to the same position (0.0 - 1.0)
     */
    public void setPosition(double position) {
        position = Math.max(0.0, Math.min(1.0, position)); // clamp safety

        servo1.setPosition(position);
        servo2.setPosition(position);
    }

    public double getPosition() {
        return servo1.getPosition();
    }
}