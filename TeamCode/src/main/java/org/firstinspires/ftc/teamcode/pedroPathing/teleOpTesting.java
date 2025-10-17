package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp
public class teleOpTesting extends LinearOpMode {

    public DcMotor LF = null;
    public DcMotor RF = null;
    public DcMotor LB = null;
    public DcMotor RB = null;

    IMU imu;

    int counter = 1;
    int counter_b = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor FL = hardwareMap.dcMotor.get("FL");
        DcMotor BL = hardwareMap.dcMotor.get("BL");
        DcMotor FR = hardwareMap.dcMotor.get("FR");
        DcMotor BR = hardwareMap.dcMotor.get("BR");


        //DcMotor sorter = hardwareMap.dcMotor.get("sorter");
        DcMotor intake = hardwareMap.dcMotor.get("intake");
        DcMotor outtakeL = hardwareMap.dcMotor.get("outtakeL");
        DcMotor outtakeR = hardwareMap.dcMotor.get("outtakeR");





        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        LF.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.FORWARD);
        LB.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.FORWARD);


        // Initialize IMU with correct orientation
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        telemetry.addLine("Ready. Press Play to start");
        telemetry.update();


        waitForStart();


        if (isStopRequested()) return;


        while (opModeIsActive()) {
            telemetry.addLine("Press A to reset Yaw");
            telemetry.addLine("Hold left bumper to drive in robot relative");
            telemetry.addLine("The left joystick sets the robot direction");
            telemetry.addLine("Moving the right joystick left and right turns the robot");

            if (gamepad1.a) {
                imu.resetYaw();
            }

            if (gamepad1.left_bumper) {
                drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            } else {
                driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            }





           /*if (gamepad2.b){
               outtakeR.setPower(1);
               outtakeL.setPower(1);
           } else {
               outtakeR.setPower(0);
               outtakeL.setPower(0);
           }
*/


            if (gamepad2.b) {
                counter = counter + 1;
            }
            if (gamepad2.a) {
                counter_b = counter_b + 1;
            }

            if (counter % 2 == 0) {
                outtakeR.setPower(1);
                outtakeL.setPower(1);


            } else if (counter % 2 == 1) {


                outtakeR.setPower(0);
                outtakeL.setPower(0);
            }


            if (counter_b % 2 == 0) {
                intake.setPower(1);
            } else if (counter % 2 == 1) {
                intake.setPower(0);
            }


            telemetry.update();
            idle(); // Let the system handle other tasks
        }




    }

    private void driveFieldRelative(double forward, double right, double rotate) {
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        theta = AngleUnit.normalizeRadians(theta - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        drive(newForward, newRight, rotate);
    }


    public void drive(double forward, double right, double rotate) {
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = Math.max(1.0,
                Math.max(Math.abs(frontLeftPower),
                        Math.max(Math.abs(frontRightPower),
                                Math.max(Math.abs(backRightPower), Math.abs(backLeftPower)))));

        double maxSpeed = 1.0; // Adjust for speed limit

        LF.setPower(maxSpeed * frontLeftPower / maxPower);
        RF.setPower(maxSpeed * frontRightPower / maxPower);
        LB.setPower(maxSpeed * backLeftPower / maxPower);
        RB.setPower(maxSpeed * backRightPower / maxPower);
    }


}


