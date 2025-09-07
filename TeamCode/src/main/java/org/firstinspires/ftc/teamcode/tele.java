package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class tele extends OpMode{
    // motors
    DcMotor fR;
    DcMotor fL;
    DcMotor bR;
    DcMotor bL;

    double drive;
    double turn;
    double strafe;

    double fLeftPow;
    double fRightPow;
    double bLeftPow;
    double bRightPow;

    @Override

    public void init() {
        fL = hardwareMap.get(DcMotor.class, "lf");
        fR = hardwareMap.get(DcMotor.class, "rf");
        bL = hardwareMap.get(DcMotor.class, "lr");
        bR = hardwareMap.get(DcMotor.class, "rr");
        bL.setDirection(DcMotorSimple.Direction.REVERSE);
        fR.setDirection(DcMotorSimple.Direction.FORWARD);
        fL.setDirection(DcMotorSimple.Direction.REVERSE);
        bR.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addData("Hardware: ", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        drive = gamepad1.left_stick_y * - 1;
        strafe = gamepad1.left_stick_x * -1;
        turn = gamepad1.right_stick_x * 1;

        ////1 is a place holder rn
        fLeftPow = Range.clip(drive + turn + strafe, -1, 1);
        bLeftPow = Range.clip(drive + turn - strafe, -1, 1);
        fRightPow = Range.clip(drive - turn - strafe, -1, 1);
        bRightPow = Range.clip(drive - turn + strafe, -1, 1);
/*
        if (gamepad1.right_trigger > 0.1) {
            fLeftPow += 1;
            bLeftPow += 1;
            fRightPow += 1;
            bRightPow += 1;
        }

        if (gamepad1.left_trigger > 0.1) {
            fLeftPow -= 0.5;
            bLeftPow -= 0.5;
            fRightPow -= 0.5;
            bRightPow -= 0.5;
        }
*/
        telemetry.addData("fL", fLeftPow);
        telemetry.addData("fR", fRightPow);
        telemetry.addData("bL", bLeftPow);
        telemetry.addData("bR", bRightPow);
        telemetry.update();

        fR.setPower(fRightPow);
        fL.setPower(fLeftPow);
        bR.setPower(bRightPow);
        bL.setPower(bLeftPow);
    }

}