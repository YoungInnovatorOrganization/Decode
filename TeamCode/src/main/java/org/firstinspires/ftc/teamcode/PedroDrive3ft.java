package org.firstinspires.ftc.teamcode;

import static java.lang.Math.atan2;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.paths.PathBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.teamcode.pedroPathing.Tuning;

@Autonomous(name = "Pedro: Drive 3ft", group = "Robot")
public class PedroDrive3ft extends LinearOpMode {
    
    public DcMotor fr, fl, br, bl;
    @Override
    public void runOpMode() throws InterruptedException{
        // Build follower from your Constants (now wired with motors + IMU)
        DcMotor fl = hardwareMap.dcMotor.get("fl");
        DcMotor fr = hardwareMap.dcMotor.get("fr");
        DcMotor bl = hardwareMap.dcMotor.get("bl");
        DcMotor br = hardwareMap.dcMotor.get("br");


        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);

        Tuning tuning = new Tuning();
        tuning.init();
        Follower follower = tuning.follower;
        // Start pose (origin, facing "forward" as defined by your frame)
        PathBuilder builder = new PathBuilder(follower);

        PathChain line1 = builder
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Pose(0, 0),
                                new Pose(0, 36)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();


        follower.setStartingPose(new Pose(0,0));
        telemetry.addLine("Ready to run line1 -> line2");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

// ---- run line1 ----
        follower.followPath(line1);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            telemetry.addData("pose", follower.getPose());
            telemetry.update();
        }
    }
}
