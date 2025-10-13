package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

/**
 * Standard Robot TeleOp for FTC using Pedro Pathing.
 * Handles robot driving, shooter motor control, and turret control.
 *
 * Author:
 *   Baron Henderson – 20077 The Indubitables (modified by Kushal Madhabhaktula)
 * Version:
 *   3.1, 10/2025
 */
@TeleOp(name = "RobotTeleop", group = "Examples")
public class RobotTeleop extends OpMode {

    private Follower follower;
    private static final double DEAD_ZONE = 0.1;


    private DcMotorEx shooterMotor;
    private Turret turret;

    private double currentTurretPose = 0.5;
    private static final double INITIAL_TURRET_POSE = 0.5;
    private static final double TURRET_INCREMENT = 0.2;

    private boolean lastRight = false;
    private boolean lastLeft = false;

    private final Pose startPose = new Pose(0, 0, 0);

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.startTeleopDrive();


        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        Servo rawTurret = hardwareMap.get(Servo.class, "servoTurret");
        turret = new Turret(rawTurret);
        turret.setTarget(INITIAL_TURRET_POSE);
        turret.apply();

        telemetry.addLine("RobotTeleop Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {

        double xInput = Math.abs(gamepad1.left_stick_x) > DEAD_ZONE ? gamepad1.left_stick_x : 0;
        double yInput = Math.abs(gamepad1.left_stick_y) > DEAD_ZONE ? -gamepad1.left_stick_y : 0;
        double turnInput = Math.abs(gamepad1.right_stick_x) > DEAD_ZONE ? gamepad1.right_stick_x : 0;

        double powerScale = gamepad1.right_trigger > 0.5 ? 0.25 : 1.0;
        follower.updateErrors();
        follower.updateVectors();
        follower.setTeleOpDrive(
                yInput * powerScale,  // forward/backward
                xInput * powerScale,  // strafe
                turnInput * powerScale, // rotation
                true                   // robot-centric
        );

        follower.update();


        if (gamepad1.a) {
            shooterMotor.setVelocity(1460);
        } else if (gamepad1.b) {
            shooterMotor.setVelocity(0);
        } else if (gamepad1.x) {
            shooterMotor.setVelocity(1000);
        } else if (gamepad1.y) {
            shooterMotor.setVelocity(1200);
        }


        boolean rightPressed = gamepad1.dpad_right;
        boolean leftPressed = gamepad1.dpad_left;

        // Only increment once per press
        if (rightPressed && !lastRight) {
            currentTurretPose += TURRET_INCREMENT;
        } else if (leftPressed && !lastLeft) {
            currentTurretPose -= TURRET_INCREMENT;
        }

        currentTurretPose = Math.max(0, Math.min(1, currentTurretPose));
        turret.setTarget(currentTurretPose);
        turret.apply();


        lastRight = rightPressed;
        lastLeft = leftPressed;

        telemetry.addData("Drive X", xInput);
        telemetry.addData("Drive Y", yInput);
        telemetry.addData("Turn", turnInput);
        telemetry.addData("Shooter RPM", shooterMotor.getVelocity());
        telemetry.addData("Turret Pos", currentTurretPose);
        telemetry.addData("Pose X", follower.getPose().getX());
        telemetry.addData("Pose Y", follower.getPose().getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void stop() {
        shooterMotor.setPower(0);
    }
}
