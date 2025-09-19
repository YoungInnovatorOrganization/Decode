package opModes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import hardwareControl.actuators.common.MotionProfiler;
import hardwareControl.actuators.common.PIDFController;


@Config
@Autonomous(name="Motion Profiled Auto OpMode", group="specimens")
public class MotionProfiledAutoOpMode extends LinearOpMode
{

    private DcMotorEx motor;

    private PIDFController pidfController;

    private MotionProfiler motionProfiler;
    private double startTime;


    public static double Kp=0.001;
    public static double Ki=0.0;
    public static double Kd=0.0;
    public static double Kf=0.0;

    public static double power=0.1;
    public static double initialPosition = 0.0;

    public static double targetPosition = -5000.0;
    public static double currentPosition=0.0;
    public static double mpMaxVelocity = 1500.0;
    public static double mpMaxAcceleration = 1000.0;

    private long count=0;

    private FtcDashboard dashboard = FtcDashboard.getInstance();
    @Override
    public void runOpMode()
    {

        telemetry = dashboard.getTelemetry();


        telemetry.addData("TestMotorOpMode", "runOpMode started");
        telemetry.update();
        waitForStart();

        motor = hardwareMap.get(DcMotorEx.class, "testMotor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Initialize PIDF controller with example gains (tune these!)

        pidfController = new PIDFController(Kp, Ki, Kd, Kf);
        pidfController.setOutputLimits(-1.0, 1.0); // Motor power limits
        pidfController.setMaxIntegralSum(100.0); // Prevent integral windup

        // Initialize motion profiler (tune these constraints!)
        motionProfiler = new MotionProfiler(mpMaxVelocity, mpMaxAcceleration); // e.g., ticks/s, ticks/s^2

        // Generate motion profile
        motionProfiler.generateProfile(initialPosition, targetPosition);

        startTime = System.nanoTime() / 1_000_000_000.0; // Start time in seconds

        while (opModeIsActive())
        {
            count++;
            telemetry.addData("TestMotorOpMode", "runOpMode while started count: %d",count);
            telemetry.update();

            // Get current time relative to start
            double currentTime = System.nanoTime() / 1_000_000_000.0 - startTime;

            // Get motion profile setpoint
            MotionProfiler.MotionState state = motionProfiler.getMotionState(currentTime);
            double setpointPosition = state.position;

            // Get current encoder position
            currentPosition = motor.getCurrentPosition();

            // Calculate motor power using PIDF
            power = pidfController.calculate(setpointPosition, currentPosition);

            // Apply power to motor
            motor.setPower(power);

            //updateDashboardGraph();

            // Telemetry for debugging

            telemetry.addData("Target Position", setpointPosition);
            telemetry.addData("Current Position", currentPosition);
            telemetry.addData("Power", power);
            telemetry.addData("Profile Time", currentTime);
            telemetry.addData("Total Time", motionProfiler.getTotalTime());
            telemetry.update();


            // Stop when motion is complete
            if (currentTime > motionProfiler.getTotalTime()) {
                motor.setPower(0);
                break;
            }
        }
    }

    private void updateDashboardGraph(){
        // --- Dashboard packet for plotting ---
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("targetPosition", targetPosition);
        packet.put("currentPosition", currentPosition);
        packet.put("power", power);

        // You can also draw a scrolling graph:
        packet.fieldOverlay()
                .setStroke("blue")
                .strokeLine(currentPosition, 0, currentPosition, 10); // example marker

        dashboard.sendTelemetryPacket(packet);

        telemetry.addData("Target", targetPosition);
        telemetry.addData("Current", currentPosition);
        telemetry.addData("Power", power);
        telemetry.update();

    }

}
