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
                new SubsystemComponent(
                       TurretOdo.INSTANCE
                ),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

    @Override
    public void onStartButtonPressed() {
        telemetry.addData("X", TurretOdo.INSTANCE.getx());
        telemetry.addData("Y", TurretOdo.INSTANCE.gety());
        telemetry.addData("TAngle", TurretOdo.INSTANCE.getTAngle());
        telemetry.addData("Heading", TurretOdo.INSTANCE.getheading());
        telemetry.update();
    }
}
