package org.firstinspires.ftc.teamcode.teleopTesting;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
@Disabled
@TeleOp(name="Intake Test")
public class IntakeTest extends OpMode {

    // Mecanum drive motors
    private DcMotor rf, rr, lf, lr;
    private CRServo rightServo, leftServo;

    // Intake motor
    private DcMotor intake;

    @Override
    public void init() {
        // Initialize mecanum drive motors
        rf = hardwareMap.get(DcMotor.class, "rf");
        rr = hardwareMap.get(DcMotor.class, "rr");
        lf = hardwareMap.get(DcMotor.class, "lf");
        lr = hardwareMap.get(DcMotor.class, "lr");

        // Initialize intake motor
        intake = hardwareMap.get(DcMotor.class, "intake");

        // Initialize CRServos
        leftServo = hardwareMap.get(CRServo.class, "leftServo");
        rightServo = hardwareMap.get(CRServo.class, "rightServo");

        // Reverse
        rf.setDirection(DcMotor.Direction.REVERSE);
        rightServo.setDirection(CRServo.Direction.REVERSE);

        telemetry.addData("Status", "Initialized - Intake Test Mode");
        telemetry.addLine();
        telemetry.addData("Controls", "Left Stick = Move");
        telemetry.addData("", "Right Stick X = Turn");
        telemetry.addData("", "RB = Forward");
        telemetry.addData("", "LB = Stop");
        telemetry.addData("", "Y = Reverse");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Mecanum drive controls - EXACT copy from last year
        double drive = -gamepad1.left_stick_y * 1.0;
        double strafe = -gamepad1.left_stick_x * 1.0;
        double turn = -gamepad1.right_stick_x * 1.0;

        double frontLeftPower = Range.clip(drive - strafe - turn, -1, 1);
        double frontRightPower = Range.clip(drive + strafe + turn, -1, 1);
        double backLeftPower = Range.clip(-drive - strafe + turn, -1, 1);
        double backRightPower = Range.clip(drive - strafe + turn, -1, 1);

        // Set drive motor powers (lf=frontLeft, rf=frontRight, lr=backLeft, rr=backRight)
        lf.setPower(frontLeftPower);
        rf.setPower(frontRightPower);
        lr.setPower(backLeftPower);
        rr.setPower(backRightPower);

        // Intake controls - three separate buttons
        if (gamepad1.right_bumper) {
            intake.setPower(1.0); // Forward
            rightServo.setPower(1.0);
            leftServo.setPower(1.0);

        }
        if (gamepad1.left_bumper) {
            intake.setPower(0.0); // Stop
            rightServo.setPower(0.0);
            leftServo.setPower(0.0);
        }
        if (gamepad1.y) {
            intake.setPower(-1.0); // Reverse
            rightServo.setPower(0.0);
            leftServo.setPower(0.0);
        }

        // Simple telemetry
        telemetry.addData("Status", "Running");
        telemetry.addData("Drive", "%.2f, %.2f, %.2f", drive, strafe, turn);

        String intakeStatus = "---";
        if (gamepad1.right_bumper) intakeStatus = "FORWARD";
        if (gamepad1.left_bumper) intakeStatus = "STOP";
        if (gamepad1.y) intakeStatus = "REVERSE";

        telemetry.addData("Intake", intakeStatus);
        telemetry.update();
    }
}