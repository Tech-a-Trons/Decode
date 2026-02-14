//package org.firstinspires.ftc.teamcode.Season.TeleOp.RegionalsPrototypeTeleops;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//@TeleOp(name = "Turret Odo TeleOp")
//public class TurretOdoTeleNoNextFTC extends OpMode {
//
//    // ========== DRIVETRAIN ==========
//    private DcMotor frontLeft;
//    private DcMotor frontRight;
//    private DcMotor backLeft;
//    private DcMotor backRight;
//
//    // ========== TURRET ==========
//    private Servo turretServo1;
//    private Servo turretServo2;
//
//    // ========== SHOOTER ==========
//    private DcMotorEx shooterMotor;
//
//    // ========== HOOD ==========
//    private Servo hoodServo;
//
//    // ========== INTAKE ==========
//    private CRServo intakeServo;
//
//    // ========== TRANSFER ==========
//    private CRServo transferServo;
//
//    // ========== COLOR SENSOR ==========
//    private ColorSensor colorSensor;
//    private int ballCount = 0;
//    private boolean lastSeenRed = false;
//
//    // ========== LED ==========
//    private Servo ledServo;  // Or however you control your RGB LED
//
//    // ========== ODOMETRY ==========
//    private DcMotor leftEncoder;
//    private DcMotor rightEncoder;
//    private DcMotor perpEncoder;  // Optional
//
//    private double x = 0;
//    private double y = 0;
//    private double heading = 0;
//
//    private int lastLeftPos = 0;
//    private int lastRightPos = 0;
//
//    // Odometry constants (TUNE THESE!)
//    private static final double TICKS_PER_INCH = 307.699557;  // Example value
//    private static final double TRACK_WIDTH = 15.0;  // Distance between wheels (inches)
//
//    // ========== TARGET ==========
//    private double targetX = 60;
//    private double targetY = 60;
//
//    // ========== TURRET PID ==========
//    private double kP = 0.02;
//    private double kI = 0.0;
//    private double kD = 0.003;
//    private double maxVelocity = 500.0;
//    private double tolerance = 2.0;
//
//    private double lastError = 0;
//    private double integral = 0;
//    private double lastTime = 0;
//    private boolean firstRun = true;
//
//    // ========== TURRET STATE ==========
//    private boolean turretAutoAimEnabled = true;
//    private double targetTurretAngle = 0;
//    private double currentTurretAngle = 0;
//
//    // ========== INTAKE STATE ==========
//    private boolean intakeToggle = false;
//
//    // ========== DRIVE MODE ==========
//    private boolean robotCentric = false;
//    private boolean togglePressed = false;
//    private double slowModeMultiplier = 0.5;
//
//    // ========== SHOOTER STATE ==========
//    private double targetShooterVelocity = 0;
//    private boolean shootRequested = false;
//
//    // ========== HOOD STATE ==========
//    private static final double CLOSE_HOOD_DISTANCE = 20.0;
//
//    // ========== TIMERS ==========
//    private ElapsedTime runtime = new ElapsedTime();
//
//    // ========== INITIALIZATION ==========
//    @Override
//    public void init() {
//        // Initialize drivetrain
//        frontLeft = hardwareMap.get(DcMotor.class, "fl");
//        frontRight = hardwareMap.get(DcMotor.class, "fr");
//        backLeft = hardwareMap.get(DcMotor.class, "bl");
//        backRight = hardwareMap.get(DcMotor.class, "br");
//
//        // Reverse left side
//        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        // Set brake mode
//        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        // Initialize turret
//        turretServo1 = hardwareMap.get(Servo.class, "turretServo1");
//        turretServo2 = hardwareMap.get(Servo.class, "turretServo2");
//        turretServo1.setPosition(0.5);
//        turretServo2.setPosition(0.5);
//
//        // Initialize shooter
//        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
//        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//
//        // Initialize hood
//        hoodServo = hardwareMap.get(Servo.class, "hood");
//        hoodServo.setPosition(0.5);
//
//        // Initialize intake
//        intakeServo = hardwareMap.get(CRServo.class, "intake");
//
//        // Initialize transfer
//        transferServo = hardwareMap.get(CRServo.class, "transfer");
//
//        // Initialize color sensor
//        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
//
//        // Initialize LED (adjust based on your hardware)
//        // ledServo = hardwareMap.get(Servo.class, "led");
//
//        // Initialize encoders (using drive motors)
//        leftEncoder = frontLeft;
//        rightEncoder = frontRight;
//
//        // Reset encoders
//        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        lastLeftPos = 0;
//        lastRightPos = 0;
//
//        // Initialize time
//        lastTime = runtime.seconds();
//        firstRun = true;
//
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
//    }
//
//    // ========== START ==========
//    @Override
//    public void start() {
//        runtime.reset();
//    }
//
//    // ========== MAIN LOOP ==========
//    @Override
//    public void loop() {
//        // ========== DRIVE MODE TOGGLE ==========
//        if (gamepad1.dpad_up && !togglePressed) {
//            robotCentric = !robotCentric;
//            togglePressed = true;
//        } else if (!gamepad1.dpad_up) {
//            togglePressed = false;
//        }
//
//        // ========== UPDATE ODOMETRY ==========
//        updateOdometry();
//
//        // ========== CALCULATE DISTANCE TO TARGET ==========
//        double distanceToTarget = Math.hypot(targetX - x, targetY - y);
//
//        // ========== TURRET AUTO-AIM TOGGLE ==========
//        if (gamepad1.b) {
//            turretAutoAimEnabled = !turretAutoAimEnabled;
//            while (gamepad1.b) {
//                // Wait for release
//            }
//        }
//
//        // ========== TURRET CONTROL ==========
//        if (turretAutoAimEnabled) {
//            autoAimTurret(distanceToTarget);
//        } else {
//            manualTurretControl();
//        }
//
//        // ========== MANUAL TARGET ADJUSTMENT ==========
//        if (gamepad1.dpad_left) {
//            turretAutoAimEnabled = false;
//            targetX -= 0.5;
//        }
//        if (gamepad1.dpad_right) {
//            turretAutoAimEnabled = false;
//            targetX += 0.5;
//        }
//
//        // ========== RESET POSE ==========
//        if (gamepad1.dpad_down) {
//            x = 0;
//            y = 0;
//            heading = 0;
//            lastLeftPos = leftEncoder.getCurrentPosition();
//            lastRightPos = rightEncoder.getCurrentPosition();
//        }
//
//        // ========== INTAKE CONTROL ==========
//        if (gamepad1.right_bumper) {
//            intakeToggle = !intakeToggle;
//            ballCount = 0;
//            while (gamepad1.right_bumper) {
//                // Wait for release
//            }
//        }
//
//        if (intakeToggle) {
//            intakeServo.setPower(1.0);
//            transferServo.setPower(0.3);
//            countBalls();
//        } else {
//            intakeServo.setPower(0.0);
//            transferServo.setPower(0.0);
//        }
//
//        // Manual intake/transfer
//        if (gamepad1.left_trigger > 0.05) {
//            intakeServo.setPower(1.0);
//            transferServo.setPower(1.0);
//        }
//
//        // Intake reverse
//        if (gamepad1.a) {
//            turretAutoAimEnabled = false;
//            intakeServo.setPower(-1.0);
//            intakeToggle = false;
//        }
//
//        // ========== SHOOTER CONTROL ==========
//        if (gamepad1.right_trigger > 0.05) {
//            shootRequested = true;
//            targetShooterVelocity = calculateShooterVelocity(distanceToTarget);
//            shooterMotor.setVelocity(targetShooterVelocity);
//        }
//
//        // Reset shooter
//        if (gamepad1.left_bumper) {
//            shooterMotor.setPower(0);
//            shootRequested = false;
//            hoodServo.setPosition(0.5);
//            intakeServo.setPower(0);
//            transferServo.setPower(0);
//        }
//
//        // ========== HOOD CONTROL ==========
//        if (distanceToTarget <= CLOSE_HOOD_DISTANCE) {
//            hoodServo.setPosition(0.2);  // Close position
//        } else {
//            double hoodPos = calculateHoodPosition(targetShooterVelocity);
//            hoodServo.setPosition(hoodPos);
//        }
//
//        // ========== DRIVETRAIN CONTROL ==========
//        double drive = -gamepad1.left_stick_y * slowModeMultiplier;
//        double strafe = gamepad1.left_stick_x * slowModeMultiplier;
//        double rotate = gamepad1.right_stick_x * slowModeMultiplier;
//
//        double frontLeftPower, frontRightPower, backLeftPower, backRightPower;
//
//        if (robotCentric) {
//            // Robot-centric drive
//            frontLeftPower = drive + strafe + rotate;
//            frontRightPower = drive - strafe - rotate;
//            backLeftPower = drive - strafe + rotate;
//            backRightPower = drive + strafe - rotate;
//        } else {
//            // Field-centric drive
//            double headingRad = heading;
//            double rotatedDrive = drive * Math.cos(-headingRad) - strafe * Math.sin(-headingRad);
//            double rotatedStrafe = drive * Math.sin(-headingRad) + strafe * Math.cos(-headingRad);
//
//            frontLeftPower = rotatedDrive + rotatedStrafe + rotate;
//            frontRightPower = rotatedDrive - rotatedStrafe - rotate;
//            backLeftPower = rotatedDrive - rotatedStrafe + rotate;
//            backRightPower = rotatedDrive + rotatedStrafe - rotate;
//        }
//
//        // Normalize powers
//        double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
//                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
//        if (maxPower > 1.0) {
//            frontLeftPower /= maxPower;
//            frontRightPower /= maxPower;
//            backLeftPower /= maxPower;
//            backRightPower /= maxPower;
//        }
//
//        frontLeft.setPower(frontLeftPower);
//        frontRight.setPower(frontRightPower);
//        backLeft.setPower(backLeftPower);
//        backRight.setPower(backRightPower);
//
//        // ========== TELEMETRY ==========
//        telemetry.addData("=== DRIVE MODE ===", "");
//        telemetry.addData("Mode", robotCentric ? "Robot Centric" : "Field Centric");
//
//        telemetry.addData("=== ROBOT POSE ===", "");
//        telemetry.addData("X", String.format("%.1f", x));
//        telemetry.addData("Y", String.format("%.1f", y));
//        telemetry.addData("Heading", String.format("%.1f°", Math.toDegrees(heading)));
//
//        telemetry.addData("=== TURRET ===", "");
//        telemetry.addData("Auto-Aim", turretAutoAimEnabled ? "ENABLED" : "DISABLED");
//        telemetry.addData("Target", "(" + String.format("%.1f", targetX) + ", " + String.format("%.1f", targetY) + ")");
//        telemetry.addData("Distance", String.format("%.1f", distanceToTarget));
//        telemetry.addData("Target Angle", String.format("%.1f°", targetTurretAngle));
//        telemetry.addData("Current Angle", String.format("%.1f°", currentTurretAngle));
//        telemetry.addData("Error", String.format("%.1f°", lastError));
//
//        telemetry.addData("=== INTAKE ===", "");
//        telemetry.addData("Intake", intakeToggle ? "ON" : "OFF");
//        telemetry.addData("Ball Count", ballCount);
//
//        telemetry.addData("=== SHOOTER ===", "");
//        telemetry.addData("Target Velocity", String.format("%.0f", targetShooterVelocity));
//        telemetry.addData("Actual Velocity", String.format("%.0f", shooterMotor.getVelocity()));
//
//        telemetry.update();
//    }
//
//    // ========== ODOMETRY UPDATE ==========
//    private void updateOdometry() {
//        int leftPos = leftEncoder.getCurrentPosition();
//        int rightPos = rightEncoder.getCurrentPosition();
//
//        int deltaLeft = leftPos - lastLeftPos;
//        int deltaRight = rightPos - lastRightPos;
//
//        lastLeftPos = leftPos;
//        lastRightPos = rightPos;
//
//        double leftInches = deltaLeft / TICKS_PER_INCH;
//        double rightInches = deltaRight / TICKS_PER_INCH;
//
//        double deltaHeading = (rightInches - leftInches) / TRACK_WIDTH;
//        heading += deltaHeading;
//
//        double forward = (leftInches + rightInches) / 2.0;
//
//        x += forward * Math.cos(heading);
//        y += forward * Math.sin(heading);
//    }
//
//    // ========== AUTO-AIM TURRET ==========
//    private void autoAimTurret(double distanceToTarget) {
//        // Calculate field-centric angle to target
//        double fieldAngleDeg = Math.toDegrees(Math.atan2(targetY - y, targetX - x));
//        fieldAngleDeg = (fieldAngleDeg + 360) % 360;
//
//        // Convert to robot-centric
//        double headingDeg = Math.toDegrees(heading);
//        headingDeg = (headingDeg + 360) % 360;
//
//        targetTurretAngle = fieldAngleDeg - headingDeg;
//        targetTurretAngle = normalizeDegrees(targetTurretAngle);
//
//        // Get current turret angle
//        currentTurretAngle = servoToAngle(turretServo1.getPosition());
//
//        // Calculate error (shortest path)
//        double error = targetTurretAngle - currentTurretAngle;
//        if (error > 180) error -= 360;
//        if (error < -180) error += 360;
//
//        // Time delta
//        double currentTime = runtime.seconds();
//        double dt = currentTime - lastTime;
//
//        if (firstRun) {
//            dt = 0.02;
//            firstRun = false;
//        }
//
//        if (dt <= 0 || dt > 0.1) {
//            dt = 0.02;
//        }
//
//        // PID calculation
//        double P_output = kP * error;
//
//        integral += error * dt;
//        if (Math.abs(error) < tolerance) {
//            integral = 0;
//        }
//        integral = clamp(integral, -100, 100);
//        double I_output = kI * integral;
//
//        double derivative = (error - lastError) / dt;
//        double D_output = kD * derivative;
//
//        double pidOutput = P_output + I_output + D_output;
//        pidOutput = clamp(pidOutput, -maxVelocity, maxVelocity);
//
//        // Update position
//        double positionChange = pidOutput * dt;
//        double newAngle = currentTurretAngle + positionChange;
//        newAngle = normalizeDegrees(newAngle);
//
//        // Set servo
//        double servoPos = angleToServo(newAngle);
//        servoPos = clamp(servoPos, 0.0, 1.0);
//
//        turretServo1.setPosition(servoPos);
//        turretServo2.setPosition(servoPos);
//
//        // Update state
//        lastError = error;
//        lastTime = currentTime;
//    }
//
//    // ========== MANUAL TURRET CONTROL ==========
//    private void manualTurretControl() {
//        double manualInput = -gamepad2.right_stick_x;
//        double currentPos = turretServo1.getPosition();
//        double newPos = currentPos + (manualInput * 0.01);
//        newPos = clamp(newPos, 0.0, 1.0);
//
//        turretServo1.setPosition(newPos);
//        turretServo2.setPosition(newPos);
//
//        currentTurretAngle = servoToAngle(newPos);
//    }
//
//    // ========== BALL COUNTING ==========
//    private void countBalls() {
//        boolean currentlySeenRed = colorSensor.red() > 200;  // Threshold
//
//        if (currentlySeenRed && !lastSeenRed) {
//            ballCount++;
//            gamepad1.rumble(100);
//        }
//
//        lastSeenRed = currentlySeenRed;
//
//        // Auto-stop at 3 balls
//        if (ballCount >= 3) {
//            intakeToggle = false;
//            gamepad1.rumble(500);
//            ballCount = 0;
//        }
//    }
//
//    // ========== SHOOTER VELOCITY CALCULATION ==========
//    private double calculateShooterVelocity(double distance) {
//        // Simple linear mapping (TUNE THESE VALUES!)
//        // Example: 20 inches = 1500 RPM, 90 inches = 2500 RPM
//        double minDist = 20;
//        double maxDist = 90;
//        double minVel = 1500;
//        double maxVel = 2500;
//
//        if (distance < minDist) return minVel;
//        if (distance > maxDist) return maxVel;
//
//        return minVel + ((distance - minDist) / (maxDist - minDist)) * (maxVel - minVel);
//    }
//
//    // ========== HOOD POSITION CALCULATION ==========
//    private double calculateHoodPosition(double velocity) {
//        // Simple linear mapping (TUNE THESE VALUES!)
//        double minVel = 1500;
//        double maxVel = 2500;
//        double minPos = 0.3;
//        double maxPos = 0.7;
//
//        if (velocity < minVel) return minPos;
//        if (velocity > maxVel) return maxPos;
//
//        return minPos + ((velocity - minVel) / (maxVel - minVel)) * (maxPos - minPos);
//    }
//
//    // ========== HELPER FUNCTIONS ==========
//    private double angleToServo(double angleDeg) {
//        angleDeg = normalizeDegrees(angleDeg);
//        double pos = 1.0 - ((angleDeg + 180) / 360.0);
//        return clamp(pos, 0.0, 1.0);
//    }
//
//    private double servoToAngle(double servoPos) {
//        double angle = 360.0 * (1.0 - servoPos) - 180.0;
//        return normalizeDegrees(angle);
//    }
//
//    private double normalizeDegrees(double angle) {
//        angle = (angle + 360) % 360;
//        if (angle > 180) angle -= 360;
//        return angle;
//    }
//
//    private double clamp(double val, double min, double max) {
//        return Math.max(min, Math.min(max, val));
//    }
//}