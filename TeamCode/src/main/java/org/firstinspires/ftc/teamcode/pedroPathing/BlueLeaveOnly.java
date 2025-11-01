package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class BlueLeaveOnly extends LinearOpMode {
    //literally just for ranking point

    private DcMotor rf, rr, lf, lr;
    private ElapsedTime runtime = new ElapsedTime();

    static final double STRAFE_SPEED = 0.5;

    @Override
    public void runOpMode() {
        rf = hardwareMap.get(DcMotor.class, "rf");
        rr = hardwareMap.get(DcMotor.class, "rr");
        lf = hardwareMap.get(DcMotor.class, "lf");
        lr = hardwareMap.get(DcMotor.class, "lr");

        rf.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        waitForStart();

        lf.setPower(-STRAFE_SPEED);
        rf.setPower(STRAFE_SPEED);
        lr.setPower(-STRAFE_SPEED);
        rr.setPower(-STRAFE_SPEED);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Strafing Left: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        lf.setPower(0);
        rf.setPower(0);
        lr.setPower(0);
        rr.setPower(0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
