package org.firstinspires.ftc.teamcode.reference.hardwareControl.actuators.claw;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardwareConfig.baseConstants.ServoConstants;
import org.firstinspires.ftc.teamcode.hardwareConfig.actuators.claw.ClawConstants;

public class ClawController {
    private static double MAX_DEGREES = 180;
    private double lastPosition;
    private Servo clawServo;
    private boolean initialized = false;

    // Private static instance (eager initialization)
    private static final ClawController INSTANCE = new ClawController();
    private LinearOpMode opMode;

    // Private constructor to prevent instantiation
    private ClawController() {
        // Initialize hardware, state, or configuration here

    }

    private static void setupConstants(){
        try {
            Class.forName(ClawConstants.class.getName());
        } catch (ClassNotFoundException e) {
            //e.printStackTrace();
        }
    }
    // Initialization method — must be called once at the beginning
    public void initialize(double startPosition, LinearOpMode opMode) {
        if (initialized) {
            return;

        }

        this.opMode = opMode;
        // This loads ClawConstants class, which runs the static block and sets ServoConstants values
       setupConstants();

        clawServo = opMode.hardwareMap.get(Servo.class, ServoConstants.name);
        clawServo.setPosition(startPosition);
        MAX_DEGREES = ServoConstants.maxDegrees;

        initialized = true;
    }

    public void reset() {
        if(initialized) {
            clawServo.setPosition(0);
            opMode.sleep(ServoConstants.waitTime);
            clawServo = null;//to be tested
            initialized = false;
        }
    }

    // Public method to access the singleton instance
    public static ClawController getInstance() {
        return INSTANCE;
    }

    // Example method

    public void moveToPosition(double position) {
        this.opMode.telemetry.addData("ClawController", "moveToPosition %.2f",position);
        this.opMode.telemetry.update();

        clawServo.setPosition(position/MAX_DEGREES);
        opMode.sleep(ServoConstants.waitTime);
        this.lastPosition = position;
    }

    public double getLastPosition(){
        return lastPosition;
    };

    public Servo.Direction getDirection(){
        return clawServo.getDirection();
    }
    public void setDirection(Servo.Direction direction){
        clawServo.setDirection(direction);
    }
}

