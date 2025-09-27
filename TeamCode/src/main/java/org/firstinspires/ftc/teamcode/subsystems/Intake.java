package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.opencv.core.Scalar;

public class Intake {

    public enum IntakeState{
        IDLE,
        INTAKING,
        DETECTED,
        EJECTING
    }
    public ColorSensor colorSensor;
    public DcMotorEx intakeMotor;
    public Telemetry telemetry;
    public IntakeState intakeState;
    private double currentThreshold = 2.0; // Calibrate this value
    private int purpleThreshold = Color.rgb(150, 0, 155);


    public Intake(HardwareMap hardwareMap, Telemetry telemetry){
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        colorSensor = hardwareMap.get(ColorSensor.class, "color sensor");
        this.telemetry = telemetry;
    }
    public void init(){
        setState(IntakeState.IDLE);
    }
    public IntakeState getCurrentState() {
        return intakeState;
    }
    public IntakeState setState(IntakeState intakeState){
        this.intakeState = intakeState;
        return intakeState;
    }
    public void periodic() {
            // The state machine logic lives in this method
            switch (intakeState) {

                case IDLE:
                    intakeMotor.setPower(0);
                    break;

                case INTAKING:
                    double current = intakeMotor.getCurrent(CurrentUnit.AMPS);

                    if (current > currentThreshold) {
                        intakeMotor.setPower(0.5);
                    }
                    break;
                case DETECTED:
                    // Stop the motor and wait for a new command
                    intakeMotor.setPower(0);
                    break;
                case EJECTING:
                    // Run the motor in reverse for a brief period
                    intakeMotor.setPower(-0.5);
                    // You would typically use a timer here to return to IDLE after a short duration
                    break;
            }
        }

        // Public methods to change the state based on driver input
        public void startIntake() {
            intakeState = IntakeState.INTAKING;
        }

        public void stopIntake() {
            intakeState = IntakeState.IDLE;
        }

        public void eject() {
            intakeState = IntakeState.EJECTING;
        }

    }

