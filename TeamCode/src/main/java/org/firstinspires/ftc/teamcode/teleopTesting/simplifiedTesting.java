package org.firstinspires.ftc.teamcode.teleopTesting;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;
@Disabled
@TeleOp(name="Simplified Testing")
public class simplifiedTesting extends OpMode {

    private DcMotor flywheelMotor;
    private DcMotor rf, rr, lf, lr;
    private DcMotor intake;
    private CRServo leftServo;
    private CRServo rightServo;

    // Adjustable speed
    private double flywheelSpeed = 0.3;  // Start at 30%
    private static final double SPEED_INCREMENT = 0.05;

    private boolean flywheelRunning = false;

    // Button debouncing
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastRB = false;
    private boolean lastLB = false;
    private boolean lastY = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    @Override
    public void init() {
        // Initialize flywheel
        flywheelMotor = hardwareMap.get(DcMotor.class, "flywheel");
        flywheelMotor.setDirection(DcMotor.Direction.REVERSE);

        // Initialize drive motors
        rf = hardwareMap.get(DcMotor.class, "rf");
        rr = hardwareMap.get(DcMotor.class, "rr");
        lf = hardwareMap.get(DcMotor.class, "lf");
        lr = hardwareMap.get(DcMotor.class, "lr");
        rf.setDirection(DcMotor.Direction.REVERSE);

        // Initialize intake
        intake = hardwareMap.get(DcMotor.class, "intake");

        // Initialize servos
        leftServo = hardwareMap.get(CRServo.class, "leftServo");
        rightServo = hardwareMap.get(CRServo.class, "rightServo");
        rightServo.setDirection(CRServo.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.addLine();
        telemetry.addLine("DRIVE: Sticks");
        telemetry.addLine("INTAKE: RB=In, LB=Stop, Y=Out");
        telemetry.addLine("FLYWHEEL: A=On, B=Off, D-pad=Speed");
        telemetry.addData("Starting Speed", "30%");
        telemetry.update();
    }

    @Override
    public void loop() {
        // === DRIVE ===
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
        if (gamepad1.right_bumper) {
            intake.setPower(1.0);
        }
        if (gamepad1.left_bumper) {
            intake.setPower(0.0);
        }
        if (gamepad1.y) {
            intake.setPower(-1.0);
        }

        // === FLYWHEEL CONTROL ===
        if (gamepad1.a && !lastA) {
            flywheelRunning = true;
        }
        if (gamepad1.b && !lastB) {
            flywheelRunning = false;
        }
        lastA = gamepad1.a;
        lastB = gamepad1.b;

        // === FLYWHEEL SPEED ADJUSTMENT ===
        if (gamepad1.dpad_up && !lastDpadUp) {
            flywheelSpeed = Math.min(1.0, flywheelSpeed + SPEED_INCREMENT);
        }
        if (gamepad1.dpad_down && !lastDpadDown) {
            flywheelSpeed = Math.max(0.0, flywheelSpeed - SPEED_INCREMENT);
        }
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;

        // === SET FLYWHEEL ===
        if (flywheelRunning) {
            flywheelMotor.setPower(flywheelSpeed);
        } else {
            flywheelMotor.setPower(0);
        }

        // === SERVOS ALWAYS RUNNING ===
        leftServo.setPower(1.0);
        rightServo.setPower(1.0);

        // === TELEMETRY ===
        telemetry.addData("Status", "Running");
        telemetry.addLine();

        String intakeStatus = "---";
        if (gamepad1.right_bumper) intakeStatus = "IN";
        if (gamepad1.left_bumper) intakeStatus = "STOP";
        if (gamepad1.y) intakeStatus = "OUT";
        telemetry.addData("Intake", intakeStatus);

        telemetry.addLine();
        if (flywheelRunning) {
            telemetry.addData("Flywheel", "🟢 ON");
            telemetry.addData("Speed", "%.0f%%", flywheelSpeed * 100);
        } else {
            telemetry.addData("Flywheel", "🔴 OFF");
            telemetry.addData("Set Speed", "%.0f%%", flywheelSpeed * 100);
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        flywheelMotor.setPower(0);
        intake.setPower(0);
        leftServo.setPower(0);
        rightServo.setPower(0);
    }
}