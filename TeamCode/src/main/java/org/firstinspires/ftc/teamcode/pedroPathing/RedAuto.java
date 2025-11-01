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

    private DcMotor intake;
    private DcMotor flywheel;
    private Servo lifter;

    private double down = 0.0;
    private double up = 0.155;

    private static final double FLYWHEEL_POWER = 0.47;
    private static final double SPINUP_TIME = 4;
    private static final double TARGET_VOLTAGE = 13.8;

    private final Pose startPose = new Pose(110, 130, Math.toRadians(90));
    private final Pose path1End = new Pose(102.44, 102.44, Math.toRadians(45));
    private final Pose path2End = new Pose(105, 93.33, Math.toRadians(0));
    private final Pose path3End = new Pose(128.5, 93.33, Math.toRadians(0));
    private final Pose exitTrianglePose = new Pose(108.5, 85, Math.toRadians(45));

    private Path firstPath, secondPath, thirdPath, returnPath, exitTrianglePath;

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        intake = hardwareMap.get(DcMotor.class, "intake");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        lifter = hardwareMap.get(Servo.class, "lifter");

        flywheel.setDirection(DcMotor.Direction.REVERSE);
        lifter.setDirection(Servo.Direction.REVERSE);

        lifter.setPosition(down);

        buildPaths();

        follower.setStartingPose(startPose);

        telemetry.addData("Status", "Initialized - RED");
        telemetry.update();
    }

    public void buildPaths() {
        firstPath = new Path(new BezierLine(startPose, path1End));
        firstPath.setLinearHeadingInterpolation(startPose.getHeading(), path1End.getHeading());

        secondPath = new Path(new BezierLine(path1End, path2End));
        secondPath.setLinearHeadingInterpolation(path1End.getHeading(), path2End.getHeading());

        thirdPath = new Path(new BezierLine(path2End, path3End));
        thirdPath.setLinearHeadingInterpolation(path2End.getHeading(), path3End.getHeading());

        returnPath = new Path(new BezierLine(path3End, path1End));
        returnPath.setLinearHeadingInterpolation(path3End.getHeading(), path1End.getHeading());

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
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(firstPath);
                double batteryVoltage1 = hardwareMap.voltageSensor.iterator().next().getVoltage();
                double compensatedPower1 = (TARGET_VOLTAGE / batteryVoltage1) * FLYWHEEL_POWER;
                compensatedPower1 = Math.min(compensatedPower1, 1.0);
                flywheel.setPower(compensatedPower1);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    setPathState(2);
                }
                break;
            case 2:
                if (pathTimer.getElapsedTimeSeconds() >= SPINUP_TIME) {
                    lifter.setPosition(up);
                    pathTimer.resetTimer();
                    setPathState(3);
                }
                break;
            case 3:
                if (pathTimer.getElapsedTimeSeconds() >= 1.0) {
                    lifter.setPosition(down);
                    pathTimer.resetTimer();
                    setPathState(4);
                }
                break;
            case 4:
                if (pathTimer.getElapsedTimeSeconds() >= 2.0) {
                    lifter.setPosition(up);
                    pathTimer.resetTimer();
                    setPathState(5);
                }
                break;
            case 5:
                if (pathTimer.getElapsedTimeSeconds() >= 1.0) {
                    lifter.setPosition(down);
                    pathTimer.resetTimer();
                    setPathState(6);
                }
                break;
            case 6:
                if (pathTimer.getElapsedTimeSeconds() >= 2.0) {
                    lifter.setPosition(up);
                    pathTimer.resetTimer();
                    setPathState(7);
                }
                break;
            case 7:
                if (pathTimer.getElapsedTimeSeconds() >= 1.0) {
                    lifter.setPosition(down);
                    flywheel.setPower(0);
                    follower.followPath(secondPath);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    setPathState(9);
                }
                break;
            case 9:
                if (pathTimer.getElapsedTimeSeconds() >= 1.0) {
                    follower.setMaxPower(0.2);
                    intake.setPower(1.0);
                    follower.followPath(thirdPath, true);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1.0);
                    pathTimer.resetTimer();
                    setPathState(11);
                }
                break;
            case 11:
                if (pathTimer.getElapsedTimeSeconds() >= 1.0) {
                    intake.setPower(0.0);
                    follower.followPath(returnPath, true);
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    double batteryVoltage2 = hardwareMap.voltageSensor.iterator().next().getVoltage();
                    double compensatedPower2 = (TARGET_VOLTAGE / batteryVoltage2) * FLYWHEEL_POWER;
                    compensatedPower2 = Math.min(compensatedPower2, 1.0);
                    flywheel.setPower(compensatedPower2);
                    pathTimer.resetTimer();
                    setPathState(13);
                }
                break;
            case 13:
                if (pathTimer.getElapsedTimeSeconds() >= SPINUP_TIME) {
                    lifter.setPosition(up);
                    pathTimer.resetTimer();
                    setPathState(14);
                }
                break;
            case 14:
                if (pathTimer.getElapsedTimeSeconds() >= 1.0) {
                    lifter.setPosition(down);
                    pathTimer.resetTimer();
                    setPathState(15);
                }
                break;
            case 15:
                if (pathTimer.getElapsedTimeSeconds() >= 1.5) {
                    lifter.setPosition(up);
                    pathTimer.resetTimer();
                    setPathState(16);
                }
                break;
            case 16:
                if (pathTimer.getElapsedTimeSeconds() >= 1.0) {
                    lifter.setPosition(down);
                    pathTimer.resetTimer();
                    setPathState(17);
                }
                break;
            case 17:
                if (pathTimer.getElapsedTimeSeconds() >= 1.5) {
                    lifter.setPosition(up);
                    pathTimer.resetTimer();
                    setPathState(18);
                }
                break;
            case 18:
                if (pathTimer.getElapsedTimeSeconds() >= 1.0) {
                    lifter.setPosition(down);
                    flywheel.setPower(0);
                    follower.followPath(exitTrianglePath, true);
                    setPathState(19);
                }
                break;
            case 19:
                if (!follower.isBusy()) {
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
        flywheel.setPower(0);
        intake.setPower(0);
    }
}
