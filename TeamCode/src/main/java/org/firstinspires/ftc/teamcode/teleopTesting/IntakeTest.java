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

    private DcMotor rf, rr, lf, lr;
    private CRServo rightServo, leftServo;
    private DcMotor intake;

    @Override
    public void init() {
        rf = hardwareMap.get(DcMotor.class, "rf");
        rr = hardwareMap.get(DcMotor.class, "rr");
        lf = hardwareMap.get(DcMotor.class, "lf");
        lr = hardwareMap.get(DcMotor.class, "lr");

        intake = hardwareMap.get(DcMotor.class, "intake");

        leftServo = hardwareMap.get(CRServo.class, "leftServo");
        rightServo = hardwareMap.get(CRServo.class, "rightServo");

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

        if (gamepad1.right_bumper) {
            intake.setPower(1.0);
            rightServo.setPower(1.0);
            leftServo.setPower(1.0);
        }
        if (gamepad1.left_bumper) {
            intake.setPower(0.0);
            rightServo.setPower(0.0);
            leftServo.setPower(0.0);
        }
        if (gamepad1.y) {
            intake.setPower(-1.0);
            rightServo.setPower(0.0);
            leftServo.setPower(0.0);
        }

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
