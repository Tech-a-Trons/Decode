package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.ServoEx;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems.BlueExperimentalDistanceLExtractor;
import org.firstinspires.ftc.teamcode.Season.Subsystems.Sensors.VoltageGet;

@Config
public class LLHood implements Subsystem {

    public static final Hood INSTANCE = new Hood();
    VoltageGet volt = new VoltageGet();
    private final ServoEx LLhood;
    private boolean isOpen = false;
    BlueExperimentalDistanceLExtractor ll;

    public LLHood(HardwareMap hardwareMap) {
        LLhood = new ServoEx("hood");
        ll = new BlueExperimentalDistanceLExtractor(hardwareMap);
        ll.startReading();
    }

    public void set(double pos) {
        LLhood.setPosition(pos);
    }

    public void HoodPower() {
        ll.startReading();
        ll.update();
        double distance = ll.getEuclideanDistance();
        set(distance/100);
        isOpen = true;
    }

//    public void close() {
//        set(closePos);
//        isOpen = false;
//    }
//
//    public void midopen() {
//        set(midopenPos);
//        isOpen = true;
//    }
//
//    public void midclose() {
//        set(midclosePos);
//        isOpen = true;
//    }

//    public void toggle() {
//        if (isOpen) close(); else open();
//    }

    @Override
    public void periodic() { }
}