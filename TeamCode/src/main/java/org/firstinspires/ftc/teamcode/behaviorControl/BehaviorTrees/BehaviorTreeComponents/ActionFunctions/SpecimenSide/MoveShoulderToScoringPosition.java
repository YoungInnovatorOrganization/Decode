package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.SpecimenSide;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.shoulder.ShoulderController;

public class MoveShoulderToScoringPosition implements ActionFunction
{
    private final LinearOpMode opMode;
    Telemetry telemetry;
    ShoulderController shoulderController;
    protected Status lastStatus = Status.FAILURE;



   double targetAngle = 90.00;
    boolean started = false;

    public MoveShoulderToScoringPosition (Telemetry telemetry, ShoulderController shoulderController, LinearOpMode opMode) {
        this.telemetry = telemetry;
        this.shoulderController = shoulderController;
        this.opMode = opMode;
        this.init();
    }

    private void init(){
        telemetry.clearAll();
    }

    public Status perform(BlackBoard blackBoard) {
        Status status;

        if (lastStatus == Status.SUCCESS) {
            return lastStatus;
        }
        if(!started){
            shoulderController.moveToTargetPosition(this.targetAngle);
            started = true;
            status =Status.RUNNING;
        } else {
            if (!shoulderController.isOnTarget()) {
             //   double currentPosition =shoulderController.getCurrentAngle();
                shoulderController.moveToTargetPosition(this.targetAngle);

                // Telemetry for debugging
           //     telemetry.addData("Target Angle", targetAngle);
           //     telemetry.addData("Current Angle", currentPosition);
            //    telemetry.update();


                status = Status.RUNNING;
            } else {
                if (shoulderController.isShoulderStuck()) {
                    status = Status.FAILURE;
                } else {
                    status = Status.SUCCESS;
                }
            }
        }
        this.telemetry.update();
        lastStatus = status;
        return status;
    }

}
