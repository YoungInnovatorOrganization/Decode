package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class CachedMotor {
    private double lastPower = 0;
    private final DcMotor motor;
    //to filter out motor commands that are too small to cause any physical
    // movement.  Its purpose is to prevent motors from "twitching" or making
    // unnecessary, tiny movements due to small fluctuations in joystick input
    // or minor errors in a control algorithm.
    private double powerThreshold = 0.01;

    public CachedMotor(DcMotor motor, double powerThreshold) {
        this.motor = motor;
        this.powerThreshold = powerThreshold;
    }

    public CachedMotor(DcMotor motor) {
        this.motor = motor;
    }

    public void setPower(double power) {
        if ((Math.abs(this.lastPower - power) > this.powerThreshold) || (power == 0 && lastPower != 0)) {
            lastPower = power;
            motor.setPower(power);
        }
    }

    public int getPosition() {
        return(motor.getCurrentPosition());
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        this.motor.setDirection(direction);
    }

    public void setCachingThreshold(double powerThreshold) {
        this.powerThreshold = powerThreshold;
    }

    public double getPower() {
        return lastPower;
    }

    public void setMode(DcMotor.RunMode runMode) {
        this.motor.setMode(runMode);
    }

    //sets the behavior for what to do when the motor power is set to zero
    //for example FLOAT the motor's internal controller does not apply any
    // resistance when power is set to zero, allowing it to spin freely with
    // its existing momentum until friction brings it to a halt.
    //BRAKE is the opposite which reverses the current which quickly stops the
    //motor's shaft.
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        this.motor.setZeroPowerBehavior(zeroPowerBehavior);
    }
}
