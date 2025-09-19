package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.SpecimenSide;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoardSingleton;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.arm.ArmController;


public class MoveArmToScoringSpecimenPosition implements ActionFunction
{
    private final LinearOpMode opMode;
    Telemetry telemetry;
    ArmController armController;
    protected Status lastStatus = Status.FAILURE;



    double targetPosition = 10.00;
    boolean started = false;

    public MoveArmToScoringSpecimenPosition (Telemetry telemetry, ArmController armderController, LinearOpMode opMode) {
        this.telemetry = telemetry;
        this.armController = armderController;
        this.opMode = opMode;
        this.init();
    }

    private void init(){
        telemetry.clearAll();
    }

    public Status perform(BlackBoardSingleton blackBoard) {
        Status status;

        if (lastStatus == Status.SUCCESS) {
            return lastStatus;
        }
        if(!started){
            armController.moveToTargetPosition(this.targetPosition);
            started = true;
            status =Status.RUNNING;
        } else {
            if (!armController.isOnTarget()) {
                //   double currentPosition =shoulderController.getCurrentAngle();
                armController.moveToTargetPosition(this.targetPosition);

                // Telemetry for debugging
                //     telemetry.addData("Target Angle", targetAngle);
                //     telemetry.addData("Current Angle", currentPosition);
                //    telemetry.update();


                status = Status.RUNNING;
            } else {
                if (armController.isArmStuck()) {
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
