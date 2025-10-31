package org.firstinspires.ftc.teamcode.teleopTesting;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Motor Test")
public class motorTest extends OpMode {

    private DcMotor flywheelMotor;

    @Override
    public void init() {
        flywheelMotor = hardwareMap.get(DcMotor.class, "flywheel");
        flywheelMotor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Ready");
        telemetry.addData("Controls", "A = Full Speed");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            flywheelMotor.setPower(1.0);
        } else {
            flywheelMotor.setPower(0);
        }

        telemetry.addData("Motor", gamepad1.a ? "RUNNING" : "STOPPED");
        telemetry.update();
    }

    @Override
    public void stop() {
        flywheelMotor.setPower(0);
    }
}