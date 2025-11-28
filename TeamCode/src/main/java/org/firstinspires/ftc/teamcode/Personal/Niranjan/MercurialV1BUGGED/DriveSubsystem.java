//package org.firstinspires.ftc.teamcode.Personal.Niranjan.MercurialV1BUGGED;
//
//import androidx.annotation.NonNull;
//
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//
//import java.lang.annotation.ElementType;
//import java.lang.annotation.Inherited;
//import java.lang.annotation.Retention;
//import java.lang.annotation.RetentionPolicy;
//import java.lang.annotation.Target;
//
//import dev.frozenmilk.dairy.core.FeatureRegistrar;
//import dev.frozenmilk.dairy.core.dependency.Dependency;
//import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
//import dev.frozenmilk.dairy.core.wrapper.Wrapper;
//import dev.frozenmilk.mercurial.commands.Lambda;
//import dev.frozenmilk.mercurial.subsystems.Subsystem;
//import dev.frozenmilk.mercurial.subsystems.SubsystemObjectCell;
//import kotlin.annotation.MustBeDocumented;
//
//public class DriveSubsystem implements Subsystem {
//
//    public static final DriveSubsystem INSTANCE = new DriveSubsystem();
//    private DriveSubsystem() {}
//
//    @Retention(RetentionPolicy.RUNTIME)
//    @Target(ElementType.TYPE)
//    @MustBeDocumented
//    @Inherited
//    public @interface Attach {}
//
//    private Dependency<?> dependency =
//            Subsystem.DEFAULT_DEPENDENCY
//                    .and(new SingleAnnotation<>(Attach.class));
//
//    @NonNull
//    @Override
//    public Dependency<?> getDependency() { return dependency; }
//
//    @Override
//    public void setDependency(@NonNull Dependency<?> dependency) {
//        this.dependency = dependency;
//    }
//
//    // Motor Cells
//    private final SubsystemObjectCell<DcMotorEx> fl = subsystemCell(() ->
//            FeatureRegistrar.getActiveOpMode().hardwareMap.get(DcMotorEx.class, "fl"));
//    private final SubsystemObjectCell<DcMotorEx> fr = subsystemCell(() ->
//            FeatureRegistrar.getActiveOpMode().hardwareMap.get(DcMotorEx.class, "fr"));
//    private final SubsystemObjectCell<DcMotorEx> bl = subsystemCell(() ->
//            FeatureRegistrar.getActiveOpMode().hardwareMap.get(DcMotorEx.class, "bl"));
//    private final SubsystemObjectCell<DcMotorEx> br = subsystemCell(() ->
//            FeatureRegistrar.getActiveOpMode().hardwareMap.get(DcMotorEx.class, "br"));
//
//    public static DcMotorEx FL() { return INSTANCE.fl.get(); }
//    public static DcMotorEx FR() { return INSTANCE.fr.get(); }
//    public static DcMotorEx BL() { return INSTANCE.bl.get(); }
//    public static DcMotorEx BR() { return INSTANCE.br.get(); }
//
//    @Override
//    public void preUserInitHook(@NonNull Wrapper opMode) {
//        // Motor direction setup
//        FL().setDirection(DcMotorSimple.Direction.REVERSE);
//        BL().setDirection(DcMotorSimple.Direction.REVERSE);
//
//        setDefaultCommand(joystickDriveCommand());
//    }
//
//    //
//    // COMMANDS
//    //
//    @NonNull
//    public static Lambda joystickDriveCommand() {
//        return new Lambda("drive")
//                .addRequirements(INSTANCE)
//                .setExecute(() -> {
//                    double drive = FeatureRegistrar.getActiveOpMode().gamepad1.left_stick_y;
//                    double strafe = FeatureRegistrar.getActiveOpMode().gamepad1.left_stick_x;
//                    double turn = FeatureRegistrar.getActiveOpMode().gamepad1.right_stick_x;
//
//                    double flPow = drive + strafe + turn;
//                    double frPow = drive - strafe - turn;
//                    double blPow = drive - strafe + turn;
//                    double brPow = drive + strafe - turn;
//
//                    FL().setPower(flPow);
//                    FR().setPower(frPow);
//                    BL().setPower(blPow);
//                    BR().setPower(brPow);
//                })
//                .setEnd(interrupted -> stopMotors());
//    }
//
//    public static void stopMotors() {
//        FL().setPower(0);
//        FR().setPower(0);
//        BL().setPower(0);
//        BR().setPower(0);
//    }
//}