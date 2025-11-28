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
//public class OuttakeSubsystem implements Subsystem {
//
//    public static final OuttakeSubsystem INSTANCE = new OuttakeSubsystem();
//    private OuttakeSubsystem() {}
//
//    @Retention(RetentionPolicy.RUNTIME)
//    @Target(ElementType.TYPE)
//    @MustBeDocumented
//    @Inherited
//    public @interface Attach{}
//
//    private Dependency<?> dependency = Subsystem.DEFAULT_DEPENDENCY
//            .and(new SingleAnnotation<>(Attach.class));
//
//    @NonNull
//    @Override
//    public Dependency<?> getDependency() { return dependency; }
//    @Override
//    public void setDependency(@NonNull Dependency<?> dependency) { this.dependency = dependency; }
//
//    private final SubsystemObjectCell<DcMotorEx> out1 = subsystemCell(() ->
//            FeatureRegistrar.getActiveOpMode().hardwareMap.get(DcMotorEx.class, "out1"));
//    private final SubsystemObjectCell<DcMotorEx> out2 = subsystemCell(() ->
//            FeatureRegistrar.getActiveOpMode().hardwareMap.get(DcMotorEx.class, "out2"));
//
//    public static DcMotorEx getOut1() { return INSTANCE.out1.get(); }
//    public static DcMotorEx getOut2() { return INSTANCE.out2.get(); }
//
//    private static final double TARGET_RPM = 3000;
//    private boolean enabled = false;
//
//    @Override
//    public void preUserInitHook(@NonNull Wrapper opMode) {
//        getOut1().setDirection(DcMotorSimple.Direction.REVERSE);
//        getOut2().setDirection(DcMotorSimple.Direction.FORWARD);
//
//        setDefaultCommand(runOuttakeCommand());
//    }
//
//    @NonNull
//    public static Lambda runOuttakeCommand() {
//        return new Lambda("outtake")
//                .addRequirements(INSTANCE)
//                .setExecute(() -> {
//                    if (INSTANCE.enabled) {
//                        double vel = INSTANCE.rpmToTicks(TARGET_RPM);
//                        getOut1().setVelocity(vel);
//                        getOut2().setVelocity(vel);
//                    }
//                })
//                .setEnd(i -> stopMotors());
//    }
//
//    public void toggle() {
//        enabled = !enabled;
//        if(!enabled) stopMotors();
//    }
//
//    // New method
//    public void stop() {
//        enabled = false;
//        stopMotors();
//    }
//
//    public static void stopMotors() {
//        getOut1().setPower(0);
//        getOut2().setPower(0);
//    }
//
//    private double rpmToTicks(double rpm) {
//        double ticksPerRev = getOut1().getMotorType().getTicksPerRev();
//        return (rpm * ticksPerRev) / 60.0;
//    }
//}