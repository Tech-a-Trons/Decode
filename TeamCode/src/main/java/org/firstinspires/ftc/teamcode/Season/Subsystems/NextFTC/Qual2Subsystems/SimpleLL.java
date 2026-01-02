package org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Season.Subsystems.LimeLightSubsystems.RedExperimentalDistanceLExtractor;

public class SimpleLL {
    private CRServo turretServo;
    private RedExperimentalDistanceLExtractor limelight;
    private Telemetry telemetry;

    // Alignment parameters
    // Alignment parameters
    private final double ALIGNMENT_THRESHOLD = 3;   // tighter
    private final double BASE_POWER = 0.03;         // was 0.01
    private final double MAX_POWER  = 1;   //1-0.8      // was 0.1

    // Proportional gain
    private final double kP = 0.005; //0.005                // was 0.3 (too aggressive with small MAX_POWER)

    // Proportional gain - increases power as error gets larger


    private boolean isAligning = false;

    public SimpleLL(HardwareMap hardwareMap, RedExperimentalDistanceLExtractor limelight) {
        this.turretServo = hardwareMap.get(CRServo.class, "turret");
        this.limelight = limelight;
    }

    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    /**
     * Aligns the turret to center on the target using tx value.
     * Logic: If tx > 1, move RIGHT. If tx < -1, move LEFT.
     */
    public void align() {
        Double tx = limelight.getTx();

        // Check if we have valid target data
        if (tx == null) {
            tx = 0.0;
            stopTurret();
        }

        isAligning = true;

        // If we're within the alignment threshold, stop
        if (Math.abs(tx) <= ALIGNMENT_THRESHOLD) {
            stopTurret();
            return;
        }

        // Determine power based on error magnitude
        double power = BASE_POWER + (kP * Math.abs(tx));

        // Clamp to max power
        if (power > MAX_POWER) {
            power = MAX_POWER;
        }

        // Determine direction based on tx value
        // If tx > 1 (target is to the RIGHT), turn RIGHT (positive power)
        // If tx < -1 (target is to the LEFT), turn LEFT (negative power)
        // Determine direction based on tx value
// If tx > 1 (target is to the RIGHT in the image), turn LEFT so the camera points right
// If tx < -1 (target is to the LEFT in the image), turn RIGHT so the camera points left
        if (tx > ALIGNMENT_THRESHOLD) {
            turretServo.setPower(-power);  // Turn LEFT
        } else if (tx < -ALIGNMENT_THRESHOLD) {
            turretServo.setPower(power);   // Turn RIGHT
        } else {
            stopTurret();
        }

    }

    /**
     * Stops turret movement
     */
    public void stopTurret() {
        turretServo.setPower(0);
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