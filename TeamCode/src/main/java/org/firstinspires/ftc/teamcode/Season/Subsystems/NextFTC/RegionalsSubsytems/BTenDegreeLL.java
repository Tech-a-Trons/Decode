package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems.BlueExperimentalDistanceLExtractor;
import org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems.RedExperimentalDistanceLExtractor;
import org.firstinspires.ftc.teamcode.Season.Subsystems.Sensors.VoltageGet;

import dev.nextftc.core.subsystems.Subsystem;

//Change this code and its counterpart after gearbox is finalized!!!!
public class BTenDegreeLL implements Subsystem {
    public ElapsedTime turrettimer;
    private Servo turretServo1;
    private Servo turretServo2;
    private BlueExperimentalDistanceLExtractor limelight;
    private Telemetry telemetry;
    private VoltageGet voltageGet;

    // Alignment parameters
    private final double ALIGNMENT_THRESHOLD = 1;  //3 // tighter
    private final double POSITION_INCREMENT = 0.001;  // Small position change per update
    private final double TX_RANGE_LIMIT = 10.0;  // Only align if tx is within ±10 degrees

    // Proportional gains for different distances
    private double kP_CLOSE = 0.01;//0.008  // For close alignmentc0.13
    private final double kP_FAR = 0.008;   // For far alignment

    // Offset for far alignment
    private final double FAR_OFFSET = 0;  // degrees offset for far shots
    private final double CLOSE_OFFSET = 0; // no offset for close shots

    private boolean isAligning = false;
    double voltage;
    private double currentPosition = 0.5;  // Track current servo position

    public BTenDegreeLL(HardwareMap hardwareMap, BlueExperimentalDistanceLExtractor limelight, VoltageGet voltageGet) {
        this.turretServo1 = hardwareMap.get(Servo.class, "turret1");
        this.turretServo2 = hardwareMap.get(Servo.class, "turret2");
        this.limelight = limelight;
        this.voltageGet = voltageGet;
        voltageGet.init(hardwareMap);
        turretServo1.setPosition(currentPosition);
        turretServo2.setPosition(-currentPosition);
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

    public void cycleAlign() {
        alignWithKpAndOffset(0.008, 4);
    }

    public void firstAlign() {
        alignWithKpAndOffset(0.008, 2);
    }

    /**
     * Aligns the turret for FAR distances (kP = 0.006, WITH 5° offset)
     */
    public void farAlign() {
        alignWithKpAndOffset(kP_FAR, FAR_OFFSET);
    }

    public void farAuto() {
        alignWithKpAndOffset(kP_FAR, -3);
    }

    /**
     * Internal alignment method with configurable kP and offset
     */
    private void alignWithKpAndOffset(double kP, double offset) {
        Double tx = limelight.getTx();
        voltage = voltageGet.getVoltage();

        // Check if we have valid target data
        if (tx == null) {
            tx = 0.0;
            stopTurret();
            return;
        }

        // Check if tx is outside the allowed range
        if (Math.abs(tx) > TX_RANGE_LIMIT && Math.abs(tx) < -TX_RANGE_LIMIT) {
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

        // Calculate position adjustment based on error
        double positionChange = kP * adjustedTx;

        // Clamp position change
        positionChange = Math.max(-POSITION_INCREMENT * 10, Math.min(POSITION_INCREMENT * 10, positionChange));

        // Update position
        currentPosition += positionChange;

        // Clamp to valid servo range [0, 1]
        currentPosition = Math.max(0.0, Math.min(1.0, currentPosition));

        turretServo1.setPosition(currentPosition);
        turretServo2.setPosition(-currentPosition);
    }

    /**
     * Manually rotate turret clockwise
     */
    public void turretRight() {
        isAligning = false;
        currentPosition = Math.min(1.0, currentPosition + 0.01);
        turretServo1.setPosition(currentPosition);
        turretServo2.setPosition(-currentPosition);
    }

    public void stopAndEnableAlign() {
        isAligning = true;
    }

    /**
     * Manually rotate turret counterclockwise
     */
    public void turretLeft() {
        isAligning = false;
        currentPosition = Math.max(0.0, currentPosition - 0.01);
        turretServo1.setPosition(currentPosition);
        turretServo2.setPosition(-currentPosition);
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
        isAligning = false;
        // Servo holds position automatically, no need to do anything
    }

    /**
     * Manually control turret with specified position
     */
    public void setTurretPower(double position) {
        isAligning = false;
        currentPosition = Math.max(0.0, Math.min(1.0, position));
        turretServo1.setPosition(currentPosition);
        turretServo2.setPosition(-currentPosition);
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
            telemetry.addData("Servo Position 1", String.format("%.3f", currentPosition));
            telemetry.addData("Servo Position 2", String.format("%.3f", currentPosition));
            telemetry.addData("Is Aligned", isAligned());
            telemetry.addData("Is Aligning", isAligning);
            telemetry.addData("Within Range", tx != null ? (Math.abs(tx) <= TX_RANGE_LIMIT) : false);
        }
    }
}