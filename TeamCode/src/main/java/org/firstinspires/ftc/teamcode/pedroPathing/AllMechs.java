package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.CommandGroup;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
import com.rowanmcalpin.nextftc.core.command.utility.SingleFunctionCommand;
import com.rowanmcalpin.nextftc.core.command.utility.conditionals.PassiveConditionalCommand;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;

import com.rowanmcalpin.nextftc.core.units.TimeSpan;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition;

public class AllMechs {

    public DcMotor leftFront, leftBack, rightFront, rightBack;
    public DcMotorEx vert_left, vert_right;
//    public DcMotorEx extension;
    public DcMotor intake;
    public Servo claw, pooper, rotate, wrist_left, wrist_right, arm_left, arm_right, hold;
    public PIDFController controller_vert;
//    public PIDFController controller_extension;
    public MultipleTelemetry telemetry;
    public Gamepad gamepad1;
    public Gamepad gamepad2;
    public IMU imu;
    public ElapsedTime timer;
    public ColorSensor colorSensor;
    // put camera stuff here if needed or put it in a different class
    public static final double POOPER_BLOCK = 1;
    public static final double POOPER_OPEN = 0;
    public static final double CLAW_OPEN = 0;
    public static final double CLAW_CLOSE = 1;
    public static double wrist_left_down = 1;
    public static double wrist_left_up = 0;
    public static double wrist_right_down = 0;
    public static double wrist_right_up = 1;
    public static double arm_left_down = 0;
    public static double arm_left_up = 1;
    public static double arm_right_down = 1;
    public static double arm_right_up = 0;
    public static double arm_left_wait = 0;
    public static double arm_right_wait = 1;
    public static double rotate_hor = 0;
    public static double rotate_vert = 0;

    //PID for elevator
    public static double p = 0, i = 0, d = 0, t = 0.075;
    public static double f = 0;
    // PID for extension
//    public static double pe = 0, ie = 0, de = 0;
//    public static double fe = 0;
    public final double ticks_in_degree = 700/180.0;
//    public static int hor_target;
    public static int vert_target;


    public AllMechs(HardwareMap hardwareMap, int left, int right, Gamepad gamepad1, Gamepad gamepad2) {
        claw = hardwareMap.get(Servo.class, "claw");
        rotate = hardwareMap.get(Servo.class, "rotate");

        wrist_left = hardwareMap.get(Servo.class, "wrist left");
        wrist_right = hardwareMap.get(Servo.class, "wrist right");

        arm_left = hardwareMap.get(Servo.class, "arm left");
        arm_right = hardwareMap.get(Servo.class, "arm right");

        pooper = hardwareMap.get(Servo.class, "pooper");
        hold = hardwareMap.get(Servo.class, "hold");

        vert_left = hardwareMap.get(DcMotorEx.class, "left elevator");
        vert_right = hardwareMap.get(DcMotorEx.class, "right elevator");

        vert_left.setDirection(DcMotorSimple.Direction.REVERSE);
        vert_right.setDirection(DcMotorSimple.Direction.REVERSE);

        vert_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vert_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        vert_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        vert_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        FilteredPIDFCoefficients vertPIDF = new FilteredPIDFCoefficients(p, i, d, f);
//        controller_vert = new PIDFController(vertPIDF);
//
//        CustomPIDFCoefficients extPIDF = new CustomPIDFCoefficients(pe, ie, de, fe);
//        controller_extension = new PIDFController(extPIDF);
//
//        extension = hardwareMap.get(DcMotorEx.class, "extension");
//
//        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront = hardwareMap.get(DcMotor.class, "front left");
        leftBack = hardwareMap.get(DcMotor.class, "rear left");
        rightFront = hardwareMap.get(DcMotor.class, "front right");
        rightBack = hardwareMap.get(DcMotor.class, "rear back");
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);


        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.telemetry = new MultipleTelemetry();
        timer = new ElapsedTime();
        colorSensor = hardwareMap.get(ColorSensor.class, "color sensor");

    }

    public Command setVertTarget(int target) {
        return new InstantCommand(() -> vert_target = target);
    }

    public Command updateVertPID() {
        return new SingleFunctionCommand(() -> {
            double rightPos = vert_right.getCurrentPosition();
            double pid = controller_vert.getError();
            double ff = Math.cos(Math.toRadians(vert_target / ticks_in_degree)) * f;
            double power = pid + ff;
            vert_left.setPower(power);
            vert_right.setPower(power);
            telemetry.addData("Vert Power", power);
            telemetry.addData("Vert Position", rightPos);
            return Math.abs(vert_target - rightPos) < 10; // Finish when within 10 ticks
        });
    }
//    public Command updateExtPID() {
//        return new SingleFunctionCommand(() -> {
//            int Pos = extension.getCurrentPosition();
//            double pid = controller_extension.getError();
//
//            double ff = Math.cos(Math.toRadians(hor_target / ticks_in_degree)) * f;
//
//            double power = pid + ff;
//
//
//            extension.setPower(power);
//
//            return true;
//        });
//    }
//    public Command setExtTarget(int extarget) {
//        return new InstantCommand(() -> hor_target = extarget);
//    }

    public Command intakeIn(){
        return new InstantCommand(()
                -> intake.setPower(-0.5));
    }

    public Command intakeBack(){
        return new InstantCommand(()
                -> intake.setPower(0.5));
    }


    public Command vertSample(){
        return new InstantCommand(()
                -> vert_target = 2700);
    }

    public Command vertDown() {
        return new InstantCommand(()
                -> vert_target = 50);
    }

//    public Command setHorTarget(int target) {
//        return new InstantCommand(() -> hor_target = target);
//    }

//    public Command horExtend() {
//        return new InstantCommand(()
//                -> hor_target = 350);
//    }

//    public Command horRetract() {
//        return new InstantCommand(()
//                -> hor_target = 20);
//    }


    public Command intakeDown() {
        return new InstantCommand(()
                -> hold.setPosition(.75));
    }

    public Command intakeUp() {
        return new InstantCommand(()
                -> hold.setPosition(.3));
    }

    public Command checkColorRed() {
        return new ParallelGroup(
                // Step 1: Set hold servo
                new InstantCommand(() -> hold.setPosition(0.75)),

                // Step 2: Check color and perform conditional actions
                new PassiveConditionalCommand(
                        () -> colorSensor.red() > colorSensor.green() + 50 && colorSensor.red() > colorSensor.blue() + 50,
                        () -> new ParallelGroup(
                                new InstantCommand(() -> pooper.setPosition(POOPER_BLOCK)),
                                new InstantCommand(() -> gamepad1.setLedColor(255, 0, 0, 5000)),
                                new InstantCommand(() -> gamepad1.rumbleBlips(1)),
                                new InstantCommand(() -> intake.setPower(0)),
                                new InstantCommand(() -> hold.setPosition(0.3))

                        ),
                        () -> new PassiveConditionalCommand(
                                () -> colorSensor.green() > colorSensor.blue() && colorSensor.red() > colorSensor.blue(),
                                () -> new ParallelGroup(
                                        new InstantCommand(() -> pooper.setPosition(POOPER_BLOCK)),
                                        new InstantCommand(() -> gamepad1.setLedColor(230, 230, 0, 5000)),
                                        new InstantCommand(() -> gamepad1.rumbleBlips(1)),
                                        new InstantCommand(() -> intake.setPower(0)),
                                        new InstantCommand(() -> hold.setPosition(0.3))

                                ),
                                () -> new PassiveConditionalCommand(
                                        () -> colorSensor.blue() > colorSensor.green() + 50 && colorSensor.blue() > colorSensor.red() + 50,
                                        () -> new ParallelGroup(
                                                new InstantCommand(() -> pooper.setPosition(POOPER_OPEN)),
                                                new InstantCommand(() -> gamepad1.setLedColor(0, 0, 255, 5000)),
                                                new InstantCommand(() -> gamepad1.rumbleBlips(1)),
                                                new InstantCommand(() -> intake.setPower(-0.65)),
                                                new Delay(TimeSpan.fromMs(500))
                                        ),
                                        () -> new PassiveConditionalCommand(
                                                ()-> gamepad1.square,
                                                () -> new ParallelGroup(
                                                        new InstantCommand(() -> pooper.setPosition(POOPER_BLOCK)),
                                                        new InstantCommand(() -> hold.setPosition(0.3)),
                                                        new InstantCommand(() -> intake.setPower(0))
                                                ),
                                                () -> new ParallelGroup(
                                                        new InstantCommand(() -> pooper.setPosition(POOPER_BLOCK)),
                                                        new InstantCommand(() -> intake.setPower(-0.65))
                                                )
                                        )
                                )
                        )
                )
        );
    }
    public Command stopIntake() {
        return new ParallelGroup(
                new InstantCommand(() -> pooper.setPosition(POOPER_BLOCK)),
                new InstantCommand(() -> intake.setPower(0))
        );
    }

    public Command armUp() {
        return new SequentialGroup(
                new InstantCommand(() -> arm_left.setPosition(arm_left_up)),
                new InstantCommand(() -> arm_right.setPosition(arm_right_up))
        );
    }
    public Command armDown() {
        return new SequentialGroup(
                new InstantCommand(() -> arm_left.setPosition(arm_left_down)),
                new InstantCommand(() -> arm_right.setPosition(arm_right_down))
        );
    }
    public Command wristUp() {
        return new ParallelGroup(
                new InstantCommand(() -> wrist_left.setPosition(wrist_left_up)),
                new InstantCommand(() -> wrist_right.setPosition(wrist_right_up))
        );
    }
    public Command wristDown() {
        return new ParallelGroup(
                new InstantCommand(() -> wrist_left.setPosition(wrist_left_down)),
                new InstantCommand(() -> wrist_right.setPosition(wrist_right_down))
        );
    }

    public Command clawClose() {
        return new InstantCommand(() -> claw.setPosition(CLAW_CLOSE));

    }
    public Command clawOpen() {
        return new InstantCommand(() -> claw.setPosition(CLAW_OPEN));

    }
    public Command rotateHor() {
        return new InstantCommand(() -> rotate.setPosition(rotate_hor));

    }
    public Command rotateVert() {
        return new InstantCommand(() -> rotate.setPosition(rotate_vert));

    }
    public Command armWait(){
        return new ParallelGroup(
                new InstantCommand(()-> arm_right.setPosition(arm_right_wait)),
                new InstantCommand(()-> arm_left.setPosition(arm_left_wait))
        );
    }
}



