package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import static java.lang.Math.atan2;

import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.Tuning;


import org.firstinspires.ftc.teamcode.pedroPathing.Tuning;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

@Autonomous
public class AutoTesting extends LinearOpMode
{
    public DcMotor fr, fl, br, bl;
//    public CRServo rFeed, lFeed;
    public void runOpMode() throws InterruptedException{

        fr = hardwareMap.dcMotor.get("fr");
        fl = hardwareMap.dcMotor.get("fl");
        br = hardwareMap.dcMotor.get("br");
        bl = hardwareMap.dcMotor.get("bl");
//        shooter = hardwareMap.dcMotor.get("flywheel1");

        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);

//        lFeed.setPower(0);
//        rFeed.setPower(0);
//        shooter.setPower(0);
        // was: org.firstinspires.ftc.teamcode.pedroPathing.Tuning.init(this);
        Tuning tuning = new Tuning();
        tuning.init();
        Follower follower = tuning.follower;

        PathBuilder builder = new PathBuilder(follower);

        PathChain line1 = builder
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Pose(18.349, 130.202),
                                new Pose(60.771, 98.495),
                                new Pose(78.826, 9.688),
                                new Pose(133.431, 7.046)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

// then run the two paths as shown above
// Start at the first control point of line1.
// Heading = tangent from the first two control points of line1.
        double h0 = atan2(86.165 - 129.321, 62.092 - 18.055); // radians
        follower.setStartingPose(new Pose(18.055, 129.321, h0));

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

//        rFeed.setPower(0.5);
//        lFeed.setPower(0.5);
//        shooter.setPower(1);

//        sleep(2000);

//        rFeed.setPower(0);
//        lFeed.setPower(0);
//        shooter.setPower(0);

    }
}

