package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

@TeleOp(name = "TeleOp", group = "Iterative Opmode")
public class Monkey4 extends OpMode {

    Drivebase driveBase = new Drivebase();

    @Override
    public void init() {
        driveBase.init(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        gamepad2.rumble(1000);
        gamepad1.rumble(1000);
    }
    @Override
    public void start() {
        driveBase.initMovement();
    }
    @Override
    public void loop() {

        driveBase.setSpeed(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_bumper, gamepad1.right_bumper                              );

    }

    @Override
    public void stop() {
        driveBase.stop();
    }

    private void rumbleGamepad2AfterSeconds(int _seconds, int _rumbleLength) {
        if (Math.abs(driveBase.returnTime().seconds() - _seconds) <= 0.1f){
            gamepad2.rumble(1000 * _rumbleLength);
        }
    }


}