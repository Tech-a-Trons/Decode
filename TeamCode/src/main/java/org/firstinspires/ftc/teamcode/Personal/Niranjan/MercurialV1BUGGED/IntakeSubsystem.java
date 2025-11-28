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
//public class IntakeSubsystem implements Subsystem {
//
//    public static final IntakeSubsystem INSTANCE = new IntakeSubsystem();
//    private IntakeSubsystem() {}
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
//    public void setDependency(@NonNull Dependency<?> dependency) { this.dependency = dependency; }
//
//    private final SubsystemObjectCell<DcMotorEx> activeIntake = subsystemCell(() ->
//            FeatureRegistrar.getActiveOpMode().hardwareMap.get(DcMotorEx.class, "activeintake"));
//
//    public static DcMotorEx getIntake() { return INSTANCE.activeIntake.get(); }
//
//    private boolean enabled = false;
//
//    @Override
//    public void preUserInitHook(@NonNull Wrapper opMode) {
//        getIntake().setDirection(DcMotorSimple.Direction.FORWARD);
//        setDefaultCommand(defaultCommand());
//    }
//
//    @NonNull
//    public Lambda defaultCommand() {
//        return new Lambda("intake default")
//                .addRequirements(this)
//                .setExecute(() -> {
//                    if (enabled) getIntake().setPower(1.0);
//                    else getIntake().setPower(0);
//                });
//    }
//
//    // Immediate control
//    public void setPower(double power) {
//        getIntake().setPower(power);
//        enabled = power != 0;
//    }
//
//    // Toggle for default command
//    public void toggle() {
//        enabled = !enabled;
//    }
//
//    // Spin intake forward
//    public void in() {
//        setPower(1.0);
//    }
//
//    // Spin intake reverse
//    public void out() {
//        setPower(-1.0);
//    }
//
//    // Stop intake
//
//    public void stop() {
//        setPower(0);
//    }
//}