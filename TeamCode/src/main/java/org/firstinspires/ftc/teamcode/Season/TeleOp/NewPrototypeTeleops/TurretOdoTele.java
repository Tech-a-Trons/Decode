package org.firstinspires.ftc.teamcode.Season.TeleOp.NewPrototypeTeleops;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Season.Pedro.Constants;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.CompliantIntake;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.Hood;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.Qual2Subsystems.TurretPID;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.TurretOdo;
import org.firstinspires.ftc.teamcode.Season.Subsystems.NextFTC.RegionalsSubsytems.TurretOdoAi;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp
public class TurretOdoTele extends NextFTCOpMode {

    public TurretOdoTele() {
        addComponents(
                new SubsystemComponent(TurretOdoAi.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

    @Override
    public void onStartButtonPressed() {
        // initialize hardware first
        TurretOdoAi.INSTANCE.init(hardwareMap);

        waitForStart(); // wait for match start

        while (opModeIsActive()) {
            TurretOdoAi.INSTANCE.periodic();

            telemetry.addData("X", TurretOdoAi.INSTANCE.getX());
            telemetry.addData("Y", TurretOdoAi.INSTANCE.getY());
            telemetry.addData("Heading", TurretOdoAi.INSTANCE.getHeading());
            telemetry.addData("TurretAngleTarget", TurretOdoAi.INSTANCE.getTurretAngleDeg());
            telemetry.addData("TurretAngleEncoder", TurretOdoAi.INSTANCE.getTurretEncoderAngleDeg());
            telemetry.update();


        }
    }
}
