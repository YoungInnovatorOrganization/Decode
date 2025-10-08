package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TeleOp - Mecanum + Intake/Outtake (Full)", group = "TeleOp")
public class TeleOpFull extends LinearOpMode {

    // --- Drive motors (mecanum) ---
    private DcMotorEx lf, rf, lb, rb;

    // --- Intake ---
    private DcMotorEx intake;

    // --- Outtake (slide + gate servo) ---
    private DcMotorEx outtake;     // e.g., linear slide / elevator
    private Servo gate;            // e.g., dump gate / claw / wrist

    // --- Configurable constants ---
    private static final double DRIVE_TURBO = 1.00;
    private static final double DRIVE_NORMAL = 0.75;
    private static final double DRIVE_SLOW = 0.40;

    private static final double INTAKE_POWER = 1.0;     // forward
    private static final double OUTTAKE_EJECT_POWER = -1.0; // reverse intake to spit out

    // Slide presets (encoder ticks). Tune these for your bot.
    private static final int SLIDE_HOME = 0;
    private static final int SLIDE_LOW = 450;
    private static final int SLIDE_MED = 900;
    private static final int SLIDE_HIGH = 1400;

    // Slide motion
    private static final double SLIDE_POWER = 1.0;
    private static final int SLIDE_TOLERANCE = 15;

    // Gate servo positions (tune to your hardware)
    private static final double GATE_CLOSED = 0.10;
    private static final double GATE_OPEN = 0.75;

    // --- Button edge detection helpers ---
    private boolean prevA, prevB, prevX, prevRB, prevLB, prevStart, prevBack;
    private boolean prevDpadUp, prevDpadDown, prevDpadLeft, prevDpadRight;
    private boolean prevY;
    private boolean intakeOnForward = false;
    private boolean intakeOnReverse = false;
    private boolean gateOpen = false;

    // Drive mode
    private enum SpeedMode { SLOW, NORMAL, TURBO }
    private SpeedMode speedMode = SpeedMode.NORMAL;

    @Override
    public void runOpMode() {
        // --- Map hardware (update names to your configuration!) ---
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        lb = hardwareMap.get(DcMotorEx.class, "lb");
        rb = hardwareMap.get(DcMotorEx.class, "rb");

        intake  = hardwareMap.get(DcMotorEx.class, "intake");
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        gate    = hardwareMap.get(Servo.class, "gate");

        // --- Set motor directions (adjust as needed) ---
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        rf.setDirection(DcMotorSimple.Direction.FORWARD);
        rb.setDirection(DcMotorSimple.Direction.FORWARD);

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        outtake.setDirection(DcMotorSimple.Direction.FORWARD);

        // --- Set zero power behavior ---
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- Outtake (slide) encoder setup ---
        outtake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake.setTargetPosition(SLIDE_HOME);
        outtake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outtake.setPower(SLIDE_POWER);

        // --- Gate initial position ---
        gate.setPosition(GATE_CLOSED);
        gateOpen = false;

        telemetry.addLine("Initialized. Check hardware names in code if anything is null.");
        telemetry.update();

        waitForStart();
        ElapsedTime runtime = new ElapsedTime();

        while (opModeIsActive()) {
            // ---------------- DRIVE (mecanum) ----------------
            // left stick: strafe (x), forward/back (y) ; right stick X: rotate
            double x = gamepad1.left_stick_x;   // strafe
            double y = -gamepad1.left_stick_y;  // forward (invert)
            double rx = gamepad1.right_stick_x; // rotate

            // choose speed mode (hold bumpers or tap Y to cycle)
            boolean lbEdge = edge(gamepad1.left_bumper, prevLB); prevLB = gamepad1.left_bumper;
            boolean rbEdge = edge(gamepad1.right_bumper, prevRB); prevRB = gamepad1.right_bumper;
            boolean yEdge  = edge(gamepad1.y, prevY);             prevY  = gamepad1.y;

            if (rbEdge) speedMode = SpeedMode.TURBO;
            else if (lbEdge) speedMode = SpeedMode.SLOW;
            if (yEdge) { // tap Y to cycle
                switch (speedMode) {
                    case SLOW:   speedMode = SpeedMode.NORMAL; break;
                    case NORMAL: speedMode = SpeedMode.TURBO;  break;
                    case TURBO:  speedMode = SpeedMode.SLOW;   break;
                }
            }

            double scale = (speedMode == SpeedMode.TURBO) ? DRIVE_TURBO
                           : (speedMode == SpeedMode.SLOW) ? DRIVE_SLOW
                           : DRIVE_NORMAL;

            // mecanum math
            double denom = Math.max(1.0, Math.abs(y) + Math.abs(x) + Math.abs(rx));
            double lfP = ((y + x + rx) / denom) * scale;
            double lbP = ((y - x + rx) / denom) * scale;
            double rfP = ((y - x - rx) / denom) * scale;
            double rbP = ((y + x - rx) / denom) * scale;

            lf.setPower(lfP);
            lb.setPower(lbP);
            rf.setPower(rfP);
            rb.setPower(rbP);

            // ---------------- INTAKE ----------------
            boolean aEdge = edge(gamepad2.a, prevA); prevA = gamepad2.a; // toggle forward
            boolean bEdge = edge(gamepad2.b, prevB); prevB = gamepad2.b; // toggle reverse
            boolean xEdge = edge(gamepad2.x, prevX); prevX = gamepad2.x; // stop

            if (aEdge) { intakeOnForward = !intakeOnForward; intakeOnReverse = false; }
            if (bEdge) { intakeOnReverse = !intakeOnReverse; intakeOnForward = false; }
            if (xEdge) { intakeOnForward = false; intakeOnReverse = false; }

            double intakePower = 0.0;
            if (intakeOnForward) intakePower = INTAKE_POWER;
            if (intakeOnReverse) intakePower = OUTTAKE_EJECT_POWER;

            // manual “hold to run” override: gamepad2.right_trigger for quick forward, left_trigger for quick reverse
            if (gamepad2.right_trigger > 0.05) {
                intakePower = INTAKE_POWER * gamepad2.right_trigger;
                intakeOnForward = intakeOnReverse = false;
            } else if (gamepad2.left_trigger > 0.05) {
                intakePower = OUTTAKE_EJECT_POWER * gamepad2.left_trigger;
                intakeOnForward = intakeOnReverse = false;
            }
            intake.setPower(intakePower);

            // ---------------- OUTTAKE (SLIDE + GATE) ----------------
            // Presets with dpad (gamepad2)
            boolean dUp    = edge(gamepad2.dpad_up, prevDpadUp);       prevDpadUp = gamepad2.dpad_up;
            boolean dDown  = edge(gamepad2.dpad_down, prevDpadDown);   prevDpadDown = gamepad2.dpad_down;
            boolean dLeft  = edge(gamepad2.dpad_left, prevDpadLeft);   prevDpadLeft = gamepad2.dpad_left;
            boolean dRight = edge(gamepad2.dpad_right, prevDpadRight); prevDpadRight = gamepad2.dpad_right;

            if (dDown) setSlideTarget(SLIDE_HOME);
            if (dLeft) setSlideTarget(SLIDE_LOW);
            if (dRight) setSlideTarget(SLIDE_MED);
            if (dUp) setSlideTarget(SLIDE_HIGH);

            // Manual nudge with left stick Y (gamepad2)
            double slideNudge = -gamepad2.left_stick_y; // up positive
            if (Math.abs(slideNudge) > 0.08) {
                // Switch to RUN_USING_ENCODER to jog, then re-arm RUN_TO_POSITION
                outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                outtake.setPower(SLIDE_POWER * slideNudge);
            } else {
                // If close to target, hold position
                if (outtake.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                    outtake.setTargetPosition(outtake.getCurrentPosition());
                    outtake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    outtake.setPower(SLIDE_POWER);
                }
            }

            // Gate toggle on RB (gamepad2), hold LB to momentary-open
            boolean gateToggleEdge = edge(gamepad2.right_bumper, prevRB); // reusing prevRB for gp2 is fine if we only care edges
            prevRB = gamepad2.right_bumper;
            if (gateToggleEdge) {
                gateOpen = !gateOpen;
            }
            boolean momentaryOpen = gamepad2.left_bumper;
            if (momentaryOpen) {
                gate.setPosition(GATE_OPEN);
            } else {
                gate.setPosition(gateOpen ? GATE_OPEN : GATE_CLOSED);
            }

            // Safety: quick slide zeroing (start+back on gamepad2)
            boolean startEdge = edge(gamepad2.start, prevStart); prevStart = gamepad2.start;
            boolean backEdge  = edge(gamepad2.back, prevBack);   prevBack  = gamepad2.back;
            if (startEdge && backEdge) {
                outtake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                outtake.setTargetPosition(SLIDE_HOME);
                outtake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                outtake.setPower(SLIDE_POWER);
            }

            // ---------------- TELEMETRY ----------------
            telemetry.addData("RunTime", "%.1fs", runtime.seconds());
            telemetry.addLine("--- Drive ---");
            telemetry.addData("Mode", speedMode);
            telemetry.addData("lf/rf/lb/rb", "%.2f / %.2f / %.2f / %.2f", lfP, rfP, lbP, rbP);

            telemetry.addLine("--- Intake ---");
            telemetry.addData("Power", "%.2f", intakePower);
            telemetry.addData("Forward?", intakeOnForward);
            telemetry.addData("Reverse?", intakeOnReverse);

            telemetry.addLine("--- Outtake ---");
            telemetry.addData("Slide pos", outtake.getCurrentPosition());
            telemetry.addData("Slide tgt", outtake.getTargetPosition());
            telemetry.addData("Gate", gateOpen ? "OPEN" : "CLOSED");

            telemetry.update();
        }

        // Stop motors on exit
        stopAll();
    }

    private boolean edge(boolean current, boolean previous) { return current && !previous; }

    private void setSlideTarget(int target) {
        outtake.setTargetPosition(target);
        if (outtake.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            outtake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        outtake.setPower(SLIDE_POWER);
    }

    private void stopAll() {
        lf.setPower(0); rf.setPower(0); lb.setPower(0); rb.setPower(0);
        intake.setPower(0);
        outtake.setPower(0);
    }
}
