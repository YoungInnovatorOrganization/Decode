package org.firstinspires.ftc.teamcode.teleopTesting;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.ArrayList;
import java.util.List;

@Disabled
@TeleOp(name="Flywheel Test (Built-in LUT)")
public class FlywheelDataCollection extends OpMode {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private DcMotorEx flywheelMotor;
    private DcMotor rf, rr, lf, lr, intake;
    private CRServo leftServo, rightServo;

    private static final int TARGET_TAG_ID = 20;
    private static final double TAG_SIZE = 8.125;
    private static final double TICKS_PER_REVOLUTION = 28.0;

    private double motorPower = 0.5;
    private static final double POWER_INCREMENT = 0.01;
    private boolean motorRunning = false;
    private boolean useLut = true;

    private boolean lastA, lastB, lastRB, lastLB, lastY, lastDpadUp, lastDpadDown, lastX;

    private final LinearLUT distanceToPower = new LinearLUT();

    @Override
    public void init() {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rf = hardwareMap.get(DcMotor.class, "rf");
        rr = hardwareMap.get(DcMotor.class, "rr");
        lf = hardwareMap.get(DcMotor.class, "lf");
        lr = hardwareMap.get(DcMotor.class, "lr");
        rf.setDirection(DcMotor.Direction.REVERSE);

        intake = hardwareMap.get(DcMotor.class, "intake");
        leftServo = hardwareMap.get(CRServo.class, "leftServo");
        rightServo = hardwareMap.get(CRServo.class, "rightServo");
        flywheelMotor.setDirection(DcMotor.Direction.REVERSE);
        rightServo.setDirection(CRServo.Direction.REVERSE);

        AprilTagLibrary.Builder lib = new AprilTagLibrary.Builder();
        lib.addTag(new AprilTagMetadata(20, "Blue Goal", TAG_SIZE, DistanceUnit.INCH));
        lib.addTag(new AprilTagMetadata(24, "Red Goal", TAG_SIZE, DistanceUnit.INCH));
        lib.addTag(new AprilTagMetadata(21, "Obelisk 1", TAG_SIZE, DistanceUnit.INCH));
        lib.addTag(new AprilTagMetadata(22, "Obelisk 2", TAG_SIZE, DistanceUnit.INCH));
        lib.addTag(new AprilTagMetadata(23, "Obelisk 3", TAG_SIZE, DistanceUnit.INCH));
        AprilTagLibrary tagLibrary = lib.build();

        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(tagLibrary)
                .setLensIntrinsics(485.856, 485.856, 444.996, 236.201)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .setCameraResolution(new Size(640, 480))
                .build();

        distanceToPower.add(12, 0.38);
        distanceToPower.add(18, 0.44);
        distanceToPower.add(24, 0.50);
        distanceToPower.add(30, 0.58);
        distanceToPower.add(36, 0.66);

        telemetry.addData("Status", "Initialized - Built-in LUT");
        telemetry.update();
    }

    @Override
    public void loop() {
        double drive = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        double fl = drive - strafe - turn;
        double fr = drive + strafe + turn;
        double bl = -drive - strafe + turn;
        double br = drive - strafe + turn;
        double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
        lf.setPower(fl / max);
        rf.setPower(fr / max);
        lr.setPower(bl / max);
        rr.setPower(br / max);

        if (gamepad1.right_bumper && !lastRB) intake.setPower(1.0);
        if (gamepad1.left_bumper && !lastLB)  intake.setPower(0.0);
        if (gamepad1.y && !lastY)             intake.setPower(-1.0);
        lastRB = gamepad1.right_bumper; lastLB = gamepad1.left_bumper; lastY = gamepad1.y;

        if (gamepad1.a && !lastA) motorRunning = true;
        if (gamepad1.b && !lastB) motorRunning = false;
        lastA = gamepad1.a; lastB = gamepad1.b;

        if (gamepad1.x && !lastX) useLut = !useLut;
        lastX = gamepad1.x;

        if (gamepad1.dpad_up && !lastDpadUp)     motorPower = Math.min(1.0, motorPower + POWER_INCREMENT);
        if (gamepad1.dpad_down && !lastDpadDown) motorPower = Math.max(0.0, motorPower - POWER_INCREMENT);
        lastDpadUp = gamepad1.dpad_up; lastDpadDown = gamepad1.dpad_down;

        boolean targetFound = false;
        double distance = 0.0;
        if (aprilTag.getDetections().size() > 0) {
            for (AprilTagDetection d : aprilTag.getDetections()) {
                if (d.id == TARGET_TAG_ID && d.metadata != null) {
                    targetFound = true;
                    distance = d.ftcPose.y;
                    break;
                }
            }
        }

        double commandedPower = motorPower;
        if (useLut && targetFound) commandedPower = Range.clip(distanceToPower.get(distance), 0.0, 1.0);
        flywheelMotor.setPower(motorRunning ? commandedPower : 0.0);

        leftServo.setPower(1.0);
        rightServo.setPower(1.0);

        double velTps = flywheelMotor.getVelocity();
        double rpm = (velTps * 60.0) / TICKS_PER_REVOLUTION;

        double voltage = 0.0;
        for (VoltageSensor vs : hardwareMap.voltageSensor) {
            voltage = Math.max(voltage, vs.getVoltage());
        }

        telemetry.addData("Target", targetFound ? "LOCKED" : "NOT VISIBLE");
        telemetry.addData("Distance", targetFound ? String.format("%.1f in", distance) : "---");
        telemetry.addData("Mode", useLut ? "LUT (auto)" : "Manual");
        telemetry.addData("Manual Power", "%.0f%%", motorPower * 100);
        telemetry.addData("Commanded Power", "%.0f%%", commandedPower * 100);
        telemetry.addData("Flywheel RPM", "%.0f", rpm);
        telemetry.addData("Battery", "%.1fV", voltage);
        telemetry.update();
    }

    @Override
    public void stop() {
        flywheelMotor.setPower(0);
        intake.setPower(0);
        leftServo.setPower(0);
        rightServo.setPower(0);
        if (visionPortal != null) visionPortal.close();
    }

    private static class LinearLUT {
        private final List<Double> xs = new ArrayList<Double>();
        private final List<Double> ys = new ArrayList<Double>();

        void add(double x, double y) {
            int i = 0;
            while (i < xs.size() && xs.get(i) < x) i++;
            xs.add(i, x);
            ys.add(i, y);
        }

        double get(double x) {
            if (xs.isEmpty()) return 0.0;
            if (x <= xs.get(0)) return ys.get(0);
            int n = xs.size();
            if (x >= xs.get(n - 1)) return ys.get(n - 1);
            for (int i = 0; i < n - 1; i++) {
                double x0 = xs.get(i), x1 = xs.get(i + 1);
                if (x >= x0 && x <= x1) {
                    double y0 = ys.get(i), y1 = ys.get(i + 1);
                    double t = (x - x0) / (x1 - x0);
                    return y0 + (y1 - y0) * t;
                }
            }
            return ys.get(n - 1);
        }
    }
}
