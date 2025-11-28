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
//public class RampSubsystem implements Subsystem {
//
//    public static final RampSubsystem INSTANCE = new RampSubsystem();
//    private RampSubsystem() {}
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
//    // Ramp motor
//    private final SubsystemObjectCell<DcMotorEx> ramp = subsystemCell(() ->
//            FeatureRegistrar.getActiveOpMode().hardwareMap.get(DcMotorEx.class, "ramp"));
//
//    public static DcMotorEx getRamp() { return INSTANCE.ramp.get(); }
//
//    private boolean enabled = false;
//    private double power = 0.5; // default ramp power
//
//    @Override
//    public void preUserInitHook(@NonNull Wrapper opMode) {
//        getRamp().setDirection(DcMotorSimple.Direction.FORWARD);
//        setDefaultCommand(rampCommand());
//    }
//
//    @NonNull
//    public static Lambda rampCommand() {
//        return new Lambda("ramp")
//                .addRequirements(INSTANCE)
//                .setExecute(() -> {
//                    if (INSTANCE.enabled) {
//                        getRamp().setPower(INSTANCE.power);
//                    }
//                })
//                .setEnd(interrupted -> getRamp().setPower(0));
//    }
//
//    public void setPower(double p) { this.power = p; }
//
//    public void toggle() { enabled = !enabled; if (!enabled) getRamp().setPower(0); }
//
//    public void stop() { enabled = false; getRamp().setPower(0); }
//}