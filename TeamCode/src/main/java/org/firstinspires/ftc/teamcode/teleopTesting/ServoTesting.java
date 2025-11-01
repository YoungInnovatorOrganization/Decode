package org.firstinspires.ftc.teamcode.teleopTesting;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoTesting extends OpMode {

    private Servo lifter;

    @Override
    public void init() {
        lifter = hardwareMap.get(Servo.class, "lifter");
        lifter.setDirection(Servo.Direction.REVERSE);
        lifter.setPosition(0.0);

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
        if (gamepad1.dpad_up){
            lifter.setPosition(0.0);
        }
        if (gamepad1.dpad_down){
            lifter.setPosition(0.155);
        }

        telemetry.addData("Status", "Running");
        telemetry.addData("lifter", lifter.getPosition());
        telemetry.update();
    }
}
