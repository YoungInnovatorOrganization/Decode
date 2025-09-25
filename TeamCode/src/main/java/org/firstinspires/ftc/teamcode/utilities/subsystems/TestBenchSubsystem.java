package org.firstinspires.ftc.teamcode.utilities.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utilities.config.hardware.StatedMotor;
import org.firstinspires.ftc.teamcode.utilities.config.hardware.cachedMotor;
import org.firstinspires.ftc.teamcode.utilities.config.core.robotConstants;

/**
 * Test Bench Subsystem - Handles PID motor and servo control
 */
public class TestBenchSubsystem {

    // Hardware components
    private StatedMotor pidMotor;
    private Servo testServo;
    private StatedMotor launcherLeftMotor;
    private StatedMotor launcherRightMotor;

    // State variables
    private double currentServoPosition = robotConstants.SERVO_HOME_POSITION;

    /**
     * Initialize the test bench hardware
     * @param hardwareMap The hardware map from the OpMode
     */
    public void init(HardwareMap hardwareMap) {
        // Initialize PID Motor
//        DcMotorEx motorEx = hardwareMap.get(DcMotorEx.class, robotConstants.PID_MOTOR_NAME);
//        pidMotor = new StatedMotor(new cachedMotor(motorEx));
        DcMotorEx launcherLMotorEx = hardwareMap.get(DcMotorEx.class, robotConstants.LAUNCH_MOTOR_L);
        launcherLeftMotor = new StatedMotor(new cachedMotor(launcherLMotorEx));
        DcMotorEx launcherRMotorEx = hardwareMap.get(DcMotorEx.class, robotConstants.LAUNCH_MOTOR_R);
        launcherRightMotor = new StatedMotor(new cachedMotor(launcherRMotorEx));

        // Configure motor for velocity control
//        pidMotor.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        pidMotor.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        pidMotor.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set PID coefficients
//        pidMotor.motor.setPIDFCoefficients(
//                robotConstants.MOTOR_KP,
//                robotConstants.MOTOR_KI,
//                robotConstants.MOTOR_KD,
//                robotConstants.MOTOR_KF
//        );

        // Initialize Servo
        testServo = hardwareMap.get(Servo.class, robotConstants.TEST_SERVO_NAME);
        testServo.setPosition(robotConstants.SERVO_HOME_POSITION);
        currentServoPosition = robotConstants.SERVO_HOME_POSITION;
    }

    public StatedMotor getMotorFromName(String name) {
        switch(name) {
            case "pidMotor": return pidMotor;
            case "launcherL": return launcherLeftMotor;
            case "launcherR": return launcherRightMotor;
        };
        return null;
    }

    /**
     * Enable PID motor at target velocity
     */
    public void enableMotor(String name) {
        StatedMotor statedMotor = getMotorFromName(name);
        statedMotor.motor.setVelocity(robotConstants.MOTOR_TARGET_VELOCITY);
        statedMotor.enabled = true;
    }

    /**
     * Disable PID motor
     */
    public void disableMotor(String name) {
        StatedMotor statedMotor = getMotorFromName(name);
        statedMotor.motor.setVelocity(0.0);
        statedMotor.enabled = false;
    }

    public void disableMotors() {
        disableMotor(robotConstants.PID_MOTOR_NAME);
        disableMotor(robotConstants.LAUNCH_MOTOR_L);
        disableMotor(robotConstants.LAUNCH_MOTOR_R);
    }

    /**
     * Set motor velocity directly
     * @param velocity Target velocity in ticks per second
     */
    public void setMotorVelocity(String name, double velocity) {
        StatedMotor statedMotor = getMotorFromName(name);
        statedMotor.motor.setVelocity(velocity);
        statedMotor.enabled = (velocity != 0.0);
    }

    /**
     * Toggle motor on/off
     */
    public void toggleMotor(String name) {
        StatedMotor statedMotor = getMotorFromName(name);
        if (statedMotor.enabled) {
            enableMotor(name);
        } else {
            disableMotor(name);
        }
    }

    /**
     * Set servo position
     * @param position Servo position (0.0 to 1.0)
     */
    public void setServoPosition(double position) {
        position = Math.max(robotConstants.SERVO_MIN_POSITION,
                Math.min(robotConstants.SERVO_MAX_POSITION, position));

        testServo.setPosition(position);
        currentServoPosition = position;
    }

    /**
     * Increment servo position
     */
    public void incrementServoPosition() {
        double newPosition = currentServoPosition + robotConstants.SERVO_INCREMENT;
        setServoPosition(newPosition);
    }

    /**
     * Decrement servo position
     */
    public void decrementServoPosition() {
        double newPosition = currentServoPosition - robotConstants.SERVO_INCREMENT;
        setServoPosition(newPosition);
    }

    /**
     * Move servo to home position
     */
    public void homeServo() {
        setServoPosition(robotConstants.SERVO_HOME_POSITION);
    }

    /**
     * Get current motor velocity
     * @return Current velocity in ticks per second
     */
    public double getMotorVelocity(String name) {
        StatedMotor statedMotor = getMotorFromName(name);
        return statedMotor.motor.getVelocity();
    }

    /**
     * Get target motor velocity
     * @return Target velocity in ticks per second
     */
    public double getTargetVelocity(String name) {
        StatedMotor statedMotor = getMotorFromName(name);
        return statedMotor.enabled ? robotConstants.MOTOR_TARGET_VELOCITY : 0.0;
    }

    /**
     * Get current servo position
     * @return Current servo position (0.0 to 1.0)
     */
    public double getServoPosition() {
        return currentServoPosition;
    }

    /**
     * Check if motor is enabled
     * @return True if motor is enabled
     */
    public boolean isMotorEnabled(String name) {
        StatedMotor statedMotor = getMotorFromName(name);
        return statedMotor.enabled;
    }

    /**
     * Get motor position in ticks
     * @return Current motor position
     */
    public int getMotorPosition(String name) {
        StatedMotor statedMotor = getMotorFromName(name);
        return statedMotor.motor.getCurrentPosition();
    }

    /**
     * Check if motor velocity is within tolerance of target
     * @return True if motor is at target velocity
     */
    public boolean isMotorAtTargetVelocity(String name) {
        StatedMotor statedMotor = getMotorFromName(name);

        double currentVelocity = getMotorVelocity(name);
        double targetVelocity = getTargetVelocity(name);
        return Math.abs(currentVelocity - targetVelocity) <= robotConstants.MOTOR_VELOCITY_TOLERANCE;
    }

    /**
     * Get telemetry data as a formatted string
     * @return Formatted telemetry string
     */
    public String getTelemetryData() {
        return String.format(
                "PID Motor: %s | Vel: %.1f/%.1f | Pos: %d | Servo: %.2f",
                pidMotor.enabled ? "ON" : "OFF",
                getMotorVelocity(robotConstants.PID_MOTOR_NAME),
                getTargetVelocity(robotConstants.PID_MOTOR_NAME),
                getMotorPosition(robotConstants.PID_MOTOR_NAME),
                currentServoPosition
        );
    }
}