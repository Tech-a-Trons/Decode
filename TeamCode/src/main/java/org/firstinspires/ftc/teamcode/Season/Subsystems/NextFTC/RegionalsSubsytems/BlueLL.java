package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems.BlueExperimentalDistanceLExtractor;
import org.firstinspires.ftc.teamcode.Season.Subsystems.Sensors.VoltageGet;

public class BlueLL {
    public ElapsedTime turrettimer;
    private CRServo turretServo;
    private BlueExperimentalDistanceLExtractor limelight;
    private Telemetry telemetry;
    private VoltageGet voltageGet;

    // Alignment parameters
    private final double ALIGNMENT_THRESHOLD = 1;  //3 // tighter
    private final double BASE_POWER = 0.03;         // was 0.03
    private final double MAX_POWER  = 0.8;   //1-0.8      // was 0.1

    // Proportional gains for different distances
    private double kP_CLOSE = 0.01;//0.008  // For close alignmentc0.13
    private final double kP_FAR = 0.008;   // For far alignment

    // Offset for far alignment
    private final double FAR_OFFSET = 0;  // degrees offset for far shots
    private final double CLOSE_OFFSET = 0; // no offset for close shots

    private boolean isAligning = false;
    double voltage;

    public BlueLL(HardwareMap hardwareMap, BlueExperimentalDistanceLExtractor limelight, VoltageGet voltageGet) {
        this.turretServo = hardwareMap.get(CRServo.class, "turret");
        this.limelight = limelight;
        this.voltageGet = voltageGet;
        voltageGet.init(hardwareMap);
    }

    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    /**
     * Aligns the turret for CLOSE distances (kP = 0.008, NO offset)
     */
    public void closeAlign() {
        alignWithKpAndOffset(kP_CLOSE, CLOSE_OFFSET);
    }
    public void cycleAlign(){alignWithKpAndOffset(0.008,-4);}
    public void firstAlign(){alignWithKpAndOffset(0.008,-2);}

    /**
     * Aligns the turret for FAR distances (kP = 0.006, WITH 5° offset)
     */
    public void farAlign() {
        alignWithKpAndOffset(kP_FAR, FAR_OFFSET);
    }
    public void farAuto() {
        alignWithKpAndOffset(kP_FAR, 0);
    }
    /**
     * Internal alignment method with configurable kP and offset
     */
    private void alignWithKpAndOffset(double kP, double offset) {
        Double tx = limelight.getTx();
        voltage = voltageGet.getVoltage();
//        if (voltage<13){
//            kP_CLOSE=0.01;
//        }
//        if (voltage>13){
//            kP_CLOSE=0.006;
//        }

        // Check if we have valid target data
        if (tx == null) {
            tx = 0.0;
            stopTurret();
            return;
        }

        isAligning = true;

        // Apply offset to tx error
        double adjustedTx = tx + offset;

        // If we're within the alignment threshold, stop
        if (Math.abs(adjustedTx) <= ALIGNMENT_THRESHOLD) {
            stopTurret();
            return;
        }

        // Determine power based on error magnitude
        double power = BASE_POWER + (kP * Math.abs(adjustedTx));

        // Clamp to max power
        if (power > MAX_POWER) {
            power = MAX_POWER;
        }

        // Determine direction based on adjusted tx value
        // If tx > 1 (target is to the RIGHT in the image), turn LEFT so the camera points right
        // If tx < -1 (target is to the LEFT in the image), turn RIGHT so the camera points left
        if (adjustedTx > ALIGNMENT_THRESHOLD) {
            turretServo.setPower(-power);  // Turn LEFT
        } else if (adjustedTx < -ALIGNMENT_THRESHOLD) {
            turretServo.setPower(power);   // Turn RIGHT
        } else {
            stopTurret();
        }
    }
    /**
     * Manually rotate turret clockwise
     */
    public void turretRight() {
        isAligning = false;
        turretServo.setPower(1);
    }
    public void stopAndEnableAlign() {
        turretServo.setPower(0);
        isAligning = true;
    }
    /**
     * Manually rotate turret counterclockwise
     */
    public void turretLeft() {
        isAligning = false;
        turretServo.setPower(-1);
    }

    /**
     * Legacy align() method - defaults to far alignment (WITH offset)
     */
    public void align() {
        farAlign();
    }

    /**
     * Stops turret movement
     */
    public void stopTurret() {
        turretServo.setPower(0);
        isAligning = false;
    }

    /**
     * Manually control turret with specified power
     */
    public void setTurretPower(double power) {
        isAligning = false;
        turretServo.setPower(power);
    }

    /**
     * Check if turret is currently aligned with target
     */
    public boolean isAligned() {
        Double tx = limelight.getTx();
        return tx != null && Math.abs(tx) <= ALIGNMENT_THRESHOLD;
    }

    /**
     * Check if turret is currently attempting to align
     */
    public boolean isAligning() {
        return isAligning;
    }

    /**
     * Update telemetry with alignment info
     */
    public void updateTelemetry() {
        if (telemetry != null) {
            Double tx = limelight.getTx();
            telemetry.addData("--- Turret Alignment ---", "");
            telemetry.addData("Target Visible", limelight.isTargetVisible());
            telemetry.addData("tx Error", tx != null ? String.format("%.2f°", tx) : "N/A");
            telemetry.addData("Is Aligned", isAligned());
            telemetry.addData("Is Aligning", isAligning);
        }
    }
}