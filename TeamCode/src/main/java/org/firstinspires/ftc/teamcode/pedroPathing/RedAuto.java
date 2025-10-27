package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Red Auto", group = "Auto")
public class RedAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    // Robot hardware
    private DcMotor intake;
    private DcMotor flywheel;
    private Servo lifter;

    // Servo positions
    private double down = 0.0;
    private double up = 0.155;

    // Flywheel constants
    private static final double FLYWHEEL_POWER = 0.47;
    private static final double SPINUP_TIME = 4; // seconds
    private static final double TARGET_VOLTAGE = 13.8; // Target voltage for compensation

    // RED SIDE POSES (corrected through testing)
    private final Pose startPose = new Pose(110, 130, Math.toRadians(90));
    private final Pose path1End = new Pose(102.44, 102.44, Math.toRadians(45));
    private final Pose path2End = new Pose(105, 93.33, Math.toRadians(0));
    private final Pose path3End = new Pose(128.5, 93.33, Math.toRadians(0));
    private final Pose exitTrianglePose = new Pose(108.5, 85, Math.toRadians(45));

    // Paths
    private Path firstPath, secondPath, thirdPath, returnPath, exitTrianglePath;

    @Override
    public void init() {
        // Initialize follower
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        // Initialize hardware
        intake = hardwareMap.get(DcMotor.class, "intake");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        lifter = hardwareMap.get(Servo.class, "lifter");

        // Set directions
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        lifter.setDirection(Servo.Direction.REVERSE);

        // Set initial servo position
        lifter.setPosition(down);

        // Build the paths
        buildPaths();

        // Set starting pose
        follower.setStartingPose(startPose);

        telemetry.addData("Status", "Initialized - RED");
        telemetry.update();
    }

    public void buildPaths() {
        // First path: start to first scoring position
        firstPath = new Path(new BezierLine(startPose, path1End));
        firstPath.setLinearHeadingInterpolation(startPose.getHeading(), path1End.getHeading());

        // Second path: first scoring position to intermediate position
        secondPath = new Path(new BezierLine(path1End, path2End));
        secondPath.setLinearHeadingInterpolation(path1End.getHeading(), path2End.getHeading());

        // Third path: intermediate position to pickup position (VERY SLOW with intake running)
        thirdPath = new Path(new BezierLine(path2End, path3End));
        thirdPath.setLinearHeadingInterpolation(path2End.getHeading(), path3End.getHeading());

        // Return path: pickup position back to shooting position
        returnPath = new Path(new BezierLine(path3End, path1End));
        returnPath.setLinearHeadingInterpolation(path3End.getHeading(), path1End.getHeading());

        // Exit triangle path: shooting position to exit triangle point (after second shooting)
        exitTrianglePath = new Path(new BezierLine(path1End, exitTrianglePose));
        exitTrianglePath.setLinearHeadingInterpolation(path1End.getHeading(), exitTrianglePose.getHeading());
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        // Update follower
        follower.update();
        autonomousPathUpdate();

        // Telemetry
        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Start following the first path and start flywheel
                follower.followPath(firstPath);
                // Apply voltage compensation to flywheel
                double batteryVoltage1 = hardwareMap.voltageSensor.iterator().next().getVoltage();
                double compensatedPower1 = (TARGET_VOLTAGE / batteryVoltage1) * FLYWHEEL_POWER;
                compensatedPower1 = Math.min(compensatedPower1, 1.0); // Safety limit
                flywheel.setPower(compensatedPower1);
                setPathState(1);
                break;
            case 1:
                // Wait for first path to complete
                if (!follower.isBusy()) {
                    // First path complete, wait 3.5 seconds for flywheel to spin up
                    pathTimer.resetTimer();
                    setPathState(2);
                }
                break;
            case 2:
                // Wait 3.5 seconds for flywheel spinup
                if (pathTimer.getElapsedTimeSeconds() >= SPINUP_TIME) {
                    // Flywheel ready, shoot first ball
                    lifter.setPosition(up);
                    pathTimer.resetTimer();
                    setPathState(3);
                }
                break;
            case 3:
                // Wait 1 second, then load next ball
                if (pathTimer.getElapsedTimeSeconds() >= 1.0) {
                    lifter.setPosition(down);
                    pathTimer.resetTimer();
                    setPathState(4);
                }
                break;
            case 4:
                // Wait 2 seconds, then shoot second ball
                if (pathTimer.getElapsedTimeSeconds() >= 2.0) {
                    lifter.setPosition(up);
                    pathTimer.resetTimer();
                    setPathState(5);
                }
                break;
            case 5:
                // Wait 1 second, then load next ball
                if (pathTimer.getElapsedTimeSeconds() >= 1.0) {
                    lifter.setPosition(down);
                    pathTimer.resetTimer();
                    setPathState(6);
                }
                break;
            case 6:
                // Wait 2 seconds, then shoot third ball
                if (pathTimer.getElapsedTimeSeconds() >= 2.0) {
                    lifter.setPosition(up);
                    pathTimer.resetTimer();
                    setPathState(7);
                }
                break;
            case 7:
                // Wait 1 second after final shot, then lower lifter and start second path
                if (pathTimer.getElapsedTimeSeconds() >= 1.0) {
                    lifter.setPosition(down);
                    flywheel.setPower(0); // Stop flywheel
                    follower.followPath(secondPath);
                    setPathState(8);
                }
                break;
            case 8:
                // Wait for second path to complete
                if (!follower.isBusy()) {
                    // Second path complete, wait 1 second
                    pathTimer.resetTimer();
                    setPathState(9);
                }
                break;
            case 9:
                // Wait 1 second after second path
                if (pathTimer.getElapsedTimeSeconds() >= 1.0) {
                    // Start slow path with intake
                    follower.setMaxPower(0.2); // Reduce speed to 20% for slow movement
                    intake.setPower(1.0); // Start intake
                    follower.followPath(thirdPath, true); // Hold at end
                    setPathState(10);
                }
                break;
            case 10:
                // Wait for third path to complete
                if (!follower.isBusy()) {
                    // Keep intake running and wait 1 second
                    // intake.setPower(1.0); - Already running from case 9
                    follower.setMaxPower(1.0); // Restore normal speed
                    pathTimer.resetTimer();
                    setPathState(11);
                }
                break;
            case 11:
                // Wait 1 second with intake still running
                if (pathTimer.getElapsedTimeSeconds() >= 1.0) {
                    // Stop intake and return to shooting position
                    intake.setPower(0.0);
                    follower.followPath(returnPath, true);
                    setPathState(12);
                }
                break;
            case 12:
                // Wait for return path to complete
                if (!follower.isBusy()) {
                    // Back at shooting position, start flywheel for second round
                    // Apply voltage compensation to flywheel
                    double batteryVoltage2 = hardwareMap.voltageSensor.iterator().next().getVoltage();
                    double compensatedPower2 = (TARGET_VOLTAGE / batteryVoltage2) * FLYWHEEL_POWER;
                    compensatedPower2 = Math.min(compensatedPower2, 1.0); // Safety limit
                    flywheel.setPower(compensatedPower2);
                    pathTimer.resetTimer();
                    setPathState(13);
                }
                break;
            case 13:
                // Wait 3.5 seconds for flywheel spinup
                if (pathTimer.getElapsedTimeSeconds() >= SPINUP_TIME) {
                    // Flywheel ready, shoot first ball (round 2)
                    lifter.setPosition(up);
                    pathTimer.resetTimer();
                    setPathState(14);
                }
                break;
            case 14:
                // Wait 1 second, then load next ball
                if (pathTimer.getElapsedTimeSeconds() >= 1.0) {
                    lifter.setPosition(down);
                    pathTimer.resetTimer();
                    setPathState(15);
                }
                break;
            case 15:
                // Wait 1.5 seconds, then shoot second ball (round 2)
                if (pathTimer.getElapsedTimeSeconds() >= 1.5) {
                    lifter.setPosition(up);
                    pathTimer.resetTimer();
                    setPathState(16);
                }
                break;
            case 16:
                // Wait 1 second, then load next ball
                if (pathTimer.getElapsedTimeSeconds() >= 1.0) {
                    lifter.setPosition(down);
                    pathTimer.resetTimer();
                    setPathState(17);
                }
                break;
            case 17:
                // Wait 1.5 seconds, then shoot third ball (round 2)
                if (pathTimer.getElapsedTimeSeconds() >= 1.5) {
                    lifter.setPosition(up);
                    pathTimer.resetTimer();
                    setPathState(18);
                }
                break;
            case 18:
                // Wait 1 second, then lower lifter and exit triangle
                if (pathTimer.getElapsedTimeSeconds() >= 1.0) {
                    lifter.setPosition(down);
                    flywheel.setPower(0); // Stop flywheel
                    follower.followPath(exitTrianglePath, true);
                    setPathState(19);
                }
                break;
            case 19:
                // Wait for exit triangle path to complete
                if (!follower.isBusy()) {
                    // All done
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init_loop() {}

    @Override
    public void stop() {
        // Stop all motors when autonomous ends
        flywheel.setPower(0);
        intake.setPower(0);
    }
}