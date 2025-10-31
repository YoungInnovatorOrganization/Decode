package org.firstinspires.ftc.teamcode.teleopTesting;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import android.util.Size;

@Disabled
@TeleOp(name="Flywheel Test")
public class FlywheelDataCollection extends OpMode {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private DcMotorEx flywheelMotor;

    // Mecanum drive motors
    private DcMotor rf, rr, lf, lr;

    // Intake motor
    private DcMotor intake;

    // Continuous rotation servos
    private CRServo leftServo;
    private CRServo rightServo;

    private static final int TARGET_TAG_ID = 20; // Blue goal
    private static final double TAG_SIZE = 8.125; // inches

    // goBILDA 5203 Yellow Jacket 1:1 Ratio (6000 RPM max)
    private static final double TICKS_PER_REVOLUTION = 28.0;

    private double motorPower = 0.5; // Start at 50% power
    private static final double POWER_INCREMENT = 0.01; // 1% increments for fine tuning
    private boolean motorRunning = false; // Track if motor is on or off

    // Button press tracking for debouncing
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastRB = false;
    private boolean lastLB = false;
    private boolean lastY = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    @Override
    public void init() {
        // Initialize flywheel motor
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Open loop - no feedback
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Initialize mecanum drive motors
        rf = hardwareMap.get(DcMotor.class, "rf");
        rr = hardwareMap.get(DcMotor.class, "rr");
        lf = hardwareMap.get(DcMotor.class, "lf");
        lr = hardwareMap.get(DcMotor.class, "lr");

        // Reverse motors
        rf.setDirection(DcMotor.Direction.REVERSE);

        // Initialize intake motor
        intake = hardwareMap.get(DcMotor.class, "intake");

        // Initialize continuous rotation servos
        leftServo = hardwareMap.get(CRServo.class, "leftServo");
        rightServo = hardwareMap.get(CRServo.class, "rightServo");

        // Reverse flywheel and right servo
        flywheelMotor.setDirection(DcMotor.Direction.REVERSE);
        rightServo.setDirection(CRServo.Direction.REVERSE);

        // Build the AprilTag library
        AprilTagLibrary.Builder libraryBuilder = new AprilTagLibrary.Builder();
        libraryBuilder.addTag(new AprilTagMetadata(20, "Blue Goal", TAG_SIZE, DistanceUnit.INCH));
        libraryBuilder.addTag(new AprilTagMetadata(24, "Red Goal", TAG_SIZE, DistanceUnit.INCH));
        libraryBuilder.addTag(new AprilTagMetadata(21, "Obelisk 1", TAG_SIZE, DistanceUnit.INCH));
        libraryBuilder.addTag(new AprilTagMetadata(22, "Obelisk 2", TAG_SIZE, DistanceUnit.INCH));
        libraryBuilder.addTag(new AprilTagMetadata(23, "Obelisk 3", TAG_SIZE, DistanceUnit.INCH));
        AprilTagLibrary tagLibrary = libraryBuilder.build();

        // Initialize AprilTag processor
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(tagLibrary)
                .setLensIntrinsics(485.856, 485.856, 444.996, 236.201)
                .build();

        // Initialize vision portal
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .setCameraResolution(new Size(640, 480))
                .build();

        telemetry.addData("Status", "Initialized - Flywheel Test");
        telemetry.addLine();
        telemetry.addLine("DRIVE: Left Stick = Move, Right Stick = Turn");
        telemetry.addLine("INTAKE: RB=Forward, LB=Stop, Y=Reverse");
        telemetry.addLine("FLYWHEEL: A=ON, B=OFF, D-pad=Speed");
        telemetry.addLine("(Servos always running)");
        telemetry.addData("Default Power", "50%");
        telemetry.update();
    }

    @Override
    public void loop() {
        // === MECANUM DRIVE ===
        double drive = -gamepad1.left_stick_y * 1.0;
        double strafe = -gamepad1.left_stick_x * 1.0;
        double turn = -gamepad1.right_stick_x * 1.0;

        double frontLeftPower = Range.clip(drive - strafe - turn, -1, 1);
        double frontRightPower = Range.clip(drive + strafe + turn, -1, 1);
        double backLeftPower = Range.clip(-drive - strafe + turn, -1, 1);
        double backRightPower = Range.clip(drive - strafe + turn, -1, 1);

        lf.setPower(frontLeftPower);
        rf.setPower(frontRightPower);
        lr.setPower(backLeftPower);
        rr.setPower(backRightPower);

        // === INTAKE ===
        if (gamepad1.right_bumper && !lastRB) {
            intake.setPower(1.0);      // Forward
        }
        if (gamepad1.left_bumper && !lastLB) {
            intake.setPower(0.0);      // Stop
        }
        if (gamepad1.y && !lastY) {
            intake.setPower(-1.0);     // Reverse
        }
        lastRB = gamepad1.right_bumper;
        lastLB = gamepad1.left_bumper;
        lastY = gamepad1.y;

        // === FLYWHEEL ON/OFF ===
        if (gamepad1.a && !lastA) {
            motorRunning = true;
        }
        if (gamepad1.b && !lastB) {
            motorRunning = false;
        }
        lastA = gamepad1.a;
        lastB = gamepad1.b;

        // === FLYWHEEL SPEED ADJUSTMENT ===
        if (gamepad1.dpad_up && !lastDpadUp) {
            motorPower = Math.min(1.0, motorPower + POWER_INCREMENT);
        }
        if (gamepad1.dpad_down && !lastDpadDown) {
            motorPower = Math.max(0.0, motorPower - POWER_INCREMENT);
        }
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;

        // === SET FLYWHEEL ===
        if (motorRunning) {
            flywheelMotor.setPower(motorPower);
        } else {
            flywheelMotor.setPower(0);
        }

        // === SERVOS ALWAYS RUNNING ===
        leftServo.setPower(1.0);
        rightServo.setPower(1.0);

        // === GET FLYWHEEL SPEED ===
        double velocityTicksPerSecond = flywheelMotor.getVelocity();
        double rpm = (velocityTicksPerSecond * 60.0) / TICKS_PER_REVOLUTION;

        // === GET BATTERY VOLTAGE ===
        double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

        // === CHECK FOR APRILTAG ===
        boolean targetFound = false;
        double distance = 0.0;

        if (aprilTag.getDetections().size() > 0) {
            for (AprilTagDetection detection : aprilTag.getDetections()) {
                if (detection.id == TARGET_TAG_ID && detection.metadata != null) {
                    targetFound = true;
                    distance = detection.ftcPose.y;
                    break;
                }
            }
        }

        // === DISPLAY TELEMETRY ===
        telemetry.addLine("════════════════════════════════");
        telemetry.addLine("     DATA COLLECTION MODE");
        telemetry.addLine("════════════════════════════════");
        telemetry.addLine();

        // Distance to target
        if (targetFound) {
            telemetry.addLine("✓ TARGET LOCKED");
            telemetry.addData("Distance", "%.2f inches", distance);
        } else {
            telemetry.addLine("✗ NO TARGET VISIBLE");
            telemetry.addData("Distance", "---");
        }
        telemetry.addLine();

        // Intake status
        String intakeStatus = "---";
        if (gamepad1.right_bumper) intakeStatus = "FORWARD";
        if (gamepad1.left_bumper) intakeStatus = "STOP";
        if (gamepad1.y) intakeStatus = "REVERSE";
        telemetry.addData("Intake", intakeStatus);
        telemetry.addLine();

        // Flywheel status
        telemetry.addData("Motor Status", motorRunning ? "🟢 ON" : "🔴 OFF");
        telemetry.addData("Set Power", "%.0f%%", motorPower * 100);
        telemetry.addData("Flywheel RPM", "%.0f", rpm);
        telemetry.addData("Servos", "ALWAYS RUNNING");
        telemetry.addData("Battery", "%.1fV", voltage);

        // Data recording prompt
        if (targetFound && motorRunning && rpm > 100) {
            telemetry.addLine();
            telemetry.addLine(">>> READY TO RECORD <<<");
            telemetry.addData("Data Point", "%.1f in → %.0f RPM", distance, rpm);
        }

        telemetry.addLine();
        telemetry.addLine("Controls: RB/LB/Y=Intake | A/B/D-pad=Flywheel");
        telemetry.update();
    }

    @Override
    public void stop() {
        flywheelMotor.setPower(0);
        intake.setPower(0);
        leftServo.setPower(0);
        rightServo.setPower(0);
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}