package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.function.Supplier;


/**
 * This is an example teleop that showcases movement and robot-centric driving.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 12/30/2024
 */

@TeleOp(name = "RobotTeleop", group = "Examples")
public class RobotTeleop extends OpMode {
    //public DriveTrain DT;

    private Follower follower;
    private static final double Kp = 0.02;
    private static final double Ki = 0.0;
    private static final double Kd = 0.002;

    double strafeHeading; // for strafing straight
    boolean strafeHeadingOn = false;

    private static double Kp_strafing = 0.002;
    private static double DEAD_ZONE = 0.1;

    private final Pose startPose = new Pose(0,0,0);
    public DcMotorEx DcMotorShooter;
    public Turret turret;
    public double initialTurretPose = 0.5;
    public double currentTurretPose = initialTurretPose;
    public double incTurret = 0.01;

    /** This method is call once when init is played, it initializes the follower **/
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.startTeleopDrive();
        follower.setStartingPose(startPose);
//        buildPath();
        DcMotorShooter = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        DcMotorShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DcMotorShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        Servo rawTurret = hardwareMap.get(Servo.class, "servoTurret");
        turret = new Turret(rawTurret);
        turret.setTarget(0.5);
        turret.apply();
    }

    public void runOpMode() throws InterruptedException {
        follower.setStartingPose(new Pose(0,0,0));
        follower.startTeleopDrive();
//        arm.setPosStarting(false);
//        wrist.setPosStarting(false);
//        clawAngle.setHorizontal();
//        waitForStart();
//        while (opModeIsActive()) {
//            follower_operate();
//            arm.operate();
//            wrist.operate();
//            slider_joystick();
//            arm_wrist_operate();
//            claw_operate();
//        robot.telemetry.update();
    }

    private void follower_operate() {
        boolean strafeOnly = false;
        double xInput = Math.abs(gamepad1.left_stick_x) > DEAD_ZONE ? gamepad1.left_stick_x : 0;
        double yInput = Math.abs(gamepad1.left_stick_y) > DEAD_ZONE ? gamepad1.left_stick_y : 0;

        if (gamepad1.right_trigger > 0.5) {
            follower.setMaxPower(0.25);
        } else if (gamepad1.left_trigger > 0.5) {
            strafeOnly = true;
            if (!strafeHeadingOn) {
                strafeHeading = follower.getPose().getHeading();
            }
            // Heading correction
            double imuHeading = follower.getPose().getHeading();
            double headingError = strafeHeading - imuHeading; // Target heading is initialized when strafing starts
            double correction = Kp_strafing * headingError;

            // Adjust movement with IMU and drift correction
            follower.update();
        } else {
            follower.setMaxPower(1.0);
        }
        if (!strafeOnly) {
            strafeHeadingOn = false;
//            follower.drive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            follower.startTeleopDrive();
        }
        follower.update();


    }
    //Code for putting auton in teleop.
    public Pose testPath = new Pose(-40,20,Math.toRadians(90));
    public PathChain randomPath;

//    public void buildPath() {
//        randomPath = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(follower.getPose()), new Point(testPath)))
//                .setLinearHeadingInterpolation(follower.getPose().getHeading(), testPath.getHeading())
//                .build();
//    }


    /** This method is called continuously after Init while waiting to be started. **/
    @Override
    public void init_loop() {
//        if(gamepad1.left_stick_button) {
//            buildPath();
//            follower.followPath(randomPath,true);
//        }
    }

    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    /** This is the main loop of the opmode and runs continuously after play **/
    @Override
    public void loop() {

        /* Update Pedro to move the robot based on:
        - Forward/Backward Movement: -gamepad1.left_stick_y
        - Left/Right Movement: -gamepad1.left_stick_x
        - Turn Left/Right Movement: -gamepad1.right_stick_x
        - Robot-Centric Mode: truei
        */
        if(gamepad1.a) {
            DcMotorShooter.setVelocity(1460);
        }
        else if (gamepad1.b) {
            DcMotorShooter.setVelocity(1400);
        }
        else if (gamepad1.x) {
            DcMotorShooter.setVelocity(1000);
        }
        else if (gamepad1.y) {
            DcMotorShooter.setVelocity(1200);
        } else if (gamepad1.dpad_right) {
            currentTurretPose += incTurret;
        } else if (gamepad1.dpad_left) {
            currentTurretPose -= incTurret;
        }
        turret.setTarget(currentTurretPose);
        turret.apply();

//        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.startTeleopDrive();
        follower.update();

        /* Telemetry Outputs of our Follower */
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
        double currentVelocity = DcMotorShooter.getVelocity();
        telemetry.addData("Current RPM:", currentVelocity);
        /* Update Telemetry to the Driver Hub */
        telemetry.update();

    }

    /** We do not use this because everything automatically should disable **/
    @Override
    public void stop() {
    }
}
