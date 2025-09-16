package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierPoint;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Pedro: Drive 3ft", group = "Pedro")
public class PedroDrive3ft extends LinearOpMode{

    private static final double FORWARD_HEADING_RAD = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        Follower follower = Constants.createFollower(hardwareMap);

        // Start pose (origin, facing "forward")
        Pose start = new Pose(0.0, 0.0, FORWARD_HEADING_RAD);
        follower.setStartingPose(start);

        // End target 36 inches forward (3 ft)
        Pose endPoint = new Pose(36.0, 0.0);

        // Pedro 2.0 accepts BezierLine(startPose, endPoint) for a straight segment.
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(start, endPoint))
                .setLinearHeadingInterpolation(start.getHeading(), start.getHeading())
                .build();


        telemetry.addLine("Ready: drive forward 36 in");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        follower.followPath(path);

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            telemetry.addData("Pose", follower.getPose());
            telemetry.update();
        }
    }
}
