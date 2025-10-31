package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(group = "1", name="teleOp")
public class teleopMeet1 extends OpMode {

    // Mecanum drive motors
    private DcMotor rf, rr, lf, lr;
    private Servo lifter;

    // Intake motor
    private DcMotor intake;

    // Flywheel motor
    private DcMotor flywheel;

    // Servo positions
    private double down = 0.0;
    private double up = 0.155;

    private static final double FLYWHEEL_POWER = 0.47;
    private static final double TARGET_VOLTAGE = 13.8; // Target voltage for compensation

    // Lifter sequence tracking
    private boolean sequenceActive = false;
    private long sequenceStartTime = 0;
    private static final long UP_DELAY = 500; // 0.5 seconds

    // Button debouncing
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastRB = false;
    private boolean lastLB = false;

    @Override
    public void init() {
        // Initialize mecanum drive motors
        rf = hardwareMap.get(DcMotor.class, "rf");
        rr = hardwareMap.get(DcMotor.class, "rr");
        lf = hardwareMap.get(DcMotor.class, "lf");
        lr = hardwareMap.get(DcMotor.class, "lr");

        // Initialize intake motor
        intake = hardwareMap.get(DcMotor.class, "intake");

        // Initialize flywheel
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");

        // Initialize Servo
        lifter = hardwareMap.get(Servo.class, "lifter");

        // Reverse
        rf.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        lifter.setDirection(Servo.Direction.REVERSE);

        // Set initial servo position
        lifter.setPosition(down);

        telemetry.addData("Status", "Initialized");
        telemetry.addLine();
        telemetry.addLine("DRIVE: Sticks");
        telemetry.addLine("INTAKE: LB=In, RB=Stop, D-Down=Out");
        telemetry.addLine("FLYWHEEL: A=Start, B=Stop");
        telemetry.addLine("LIFTER: D-Pad Up (Auto Up->Down)");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Get current battery voltage
        double batteryVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

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

        // === INTAKE CONTROLS ===
        // Left bumper = In
        if (gamepad1.left_bumper && !lastLB) {
            intake.setPower(1.0);
        }

        // Right bumper = Stop
        if (gamepad1.right_bumper && !lastRB) {
            intake.setPower(0.0);
        }

        // D-pad Down = Out
        if (gamepad1.dpad_down && !lastDpadDown) {
            intake.setPower(-1.0);
        }

        lastLB = gamepad1.left_bumper;
        lastRB = gamepad1.right_bumper;
        lastDpadDown = gamepad1.dpad_down;

        // === FLYWHEEL CONTROL WITH VOLTAGE COMPENSATION ===
        // A = Start flywheel
        if (gamepad1.a && !lastA) {
            double compensatedPower = (TARGET_VOLTAGE / batteryVoltage) * FLYWHEEL_POWER;
            compensatedPower = Range.clip(compensatedPower, 0, 1.0); // Safety limit
            flywheel.setPower(compensatedPower);
        }

        // B = Stop flywheel
        if (gamepad1.b && !lastB) {
            flywheel.setPower(0);
        }

        lastA = gamepad1.a;
        lastB = gamepad1.b;

        // === LIFTER AUTO-SEQUENCE ===
        // D-pad Up = Start sequence (up -> wait -> down)
        if (gamepad1.dpad_up && !lastDpadUp && !sequenceActive) {
            lifter.setPosition(up);
            sequenceStartTime = System.currentTimeMillis();
            sequenceActive = true;
        }

        lastDpadUp = gamepad1.dpad_up;

        // Handle sequence timing
        if (sequenceActive) {
            long elapsed = System.currentTimeMillis() - sequenceStartTime;
            if (elapsed >= UP_DELAY) {
                lifter.setPosition(down);
                sequenceActive = false;
            }
        }

        // === TELEMETRY ===
        telemetry.addData("Status", "Running");
        telemetry.addData("Battery Voltage", "%.2fV", batteryVoltage);
        telemetry.addData("Drive", "%.2f, %.2f, %.2f", drive, strafe, turn);
        telemetry.addData("Intake Power", "%.2f", intake.getPower());
        telemetry.addData("Flywheel Power", "%.2f", flywheel.getPower());
        telemetry.addData("Lifter Position", "%.3f", lifter.getPosition());
        if (sequenceActive) {
            telemetry.addData("Sequence", "ACTIVE (%.1fs)", (System.currentTimeMillis() - sequenceStartTime) / 1000.0);
        }
        telemetry.update();
    }
}