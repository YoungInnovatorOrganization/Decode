package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(group = "1", name="teleOp")
public class teleopMeet1 extends OpMode {

    private DcMotor rf, rr, lf, lr;
    private Servo lifter;
    private DcMotor intake;
    private DcMotor flywheel;

    private double down = 0.0;
    private double up = 0.155;

    private static final double FLYWHEEL_POWER = 0.5;
    private static final double TARGET_VOLTAGE = 13.8;

    private boolean sequenceActive = false;
    private long sequenceStartTime = 0;
    private static final long UP_DELAY = 500;

    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastRB = false;
    private boolean lastLB = false;

    private double desiredFlywheelCmd = 0.0;

    @Override
    public void init() {
        rf = hardwareMap.get(DcMotor.class, "rf");
        rr = hardwareMap.get(DcMotor.class, "rr");
        lf = hardwareMap.get(DcMotor.class, "lf");
        lr = hardwareMap.get(DcMotor.class, "lr");
        intake = hardwareMap.get(DcMotor.class, "intake");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        lifter = hardwareMap.get(Servo.class, "lifter");

        rf.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        lifter.setDirection(Servo.Direction.REVERSE);

        lifter.setPosition(down);

        telemetry.addData("Status", "Initialized");
        telemetry.addLine();
        telemetry.addLine("DRIVE: Sticks");
        telemetry.addLine("INTAKE: LB=In, RB=Stop, D-Down=Out");
        telemetry.addLine("FLYWHEEL: A=Start, B=Stop, X=Reverse");
        telemetry.addLine("LIFTER: D-Pad Up (Auto Up->Down)");
        telemetry.update();
    }

    @Override
    public void loop() {
        double batteryVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

        double drive = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        double frontLeftPower = Range.clip(drive - strafe - turn, -1, 1);
        double frontRightPower = Range.clip(drive + strafe + turn, -1, 1);
        double backLeftPower = Range.clip(-drive - strafe + turn, -1, 1);
        double backRightPower = Range.clip(drive - strafe + turn, -1, 1);

        lf.setPower(frontLeftPower);
        rf.setPower(frontRightPower);
        lr.setPower(backLeftPower);
        rr.setPower(backRightPower);

        if (gamepad1.left_bumper && !lastLB) {
            intake.setPower(1.0);
        }
        if (gamepad1.right_bumper && !lastRB) {
            intake.setPower(0.0);
        }
        if (gamepad1.dpad_down && !lastDpadDown) {
            intake.setPower(-1.0);
        }
        lastLB = gamepad1.left_bumper;
        lastRB = gamepad1.right_bumper;
        lastDpadDown = gamepad1.dpad_down;

        if (gamepad1.a && !lastA) {
            desiredFlywheelCmd = FLYWHEEL_POWER;
        }
        if (gamepad1.x) {
            desiredFlywheelCmd = -0.5;
        }
        if (gamepad1.b && !lastB) {
            desiredFlywheelCmd = 0.0;
        }
        lastA = gamepad1.a;
        lastB = gamepad1.b;

        double compensated = (batteryVoltage > 0) ? (TARGET_VOLTAGE / batteryVoltage) * Math.abs(desiredFlywheelCmd) : 0.0;
        compensated = Range.clip(compensated, 0.0, 1.0);
        double output = (desiredFlywheelCmd >= 0) ? compensated : -compensated;
        flywheel.setPower(output);

        if (gamepad1.dpad_up && !lastDpadUp && !sequenceActive) {
            lifter.setPosition(up);
            sequenceStartTime = System.currentTimeMillis();
            sequenceActive = true;
        }
        lastDpadUp = gamepad1.dpad_up;

        if (sequenceActive) {
            long elapsed = System.currentTimeMillis() - sequenceStartTime;
            if (elapsed >= UP_DELAY) {
                lifter.setPosition(down);
                sequenceActive = false;
            }
        }

        telemetry.addData("Status", "Running");
        telemetry.addData("Battery Voltage", "%.2fV", batteryVoltage);
        telemetry.addData("Drive", "%.2f, %.2f, %.2f", drive, strafe, turn);
        telemetry.addData("Intake Power", "%.2f", intake.getPower());
        telemetry.addData("Flywheel Cmd/Out", "%.2f / %.2f", desiredFlywheelCmd, flywheel.getPower());
        telemetry.addData("Lifter Position", "%.3f", lifter.getPosition());
        if (sequenceActive) {
            telemetry.addData("Sequence", "ACTIVE (%.1fs)", (System.currentTimeMillis() - sequenceStartTime) / 1000.0);
        }
        telemetry.update();
    }
}
