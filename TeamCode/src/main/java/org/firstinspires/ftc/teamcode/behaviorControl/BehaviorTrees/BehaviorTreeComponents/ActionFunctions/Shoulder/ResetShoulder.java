package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Shoulder;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoardSingleton;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.shoulder.ShoulderController;

public class ResetShoulder implements ActionFunction
{
    private final LinearOpMode opMode;
    Telemetry telemetry;
    ShoulderController shoulderController;
    protected Status lastStatus = Status.FAILURE;

    boolean started = false;

    public ResetShoulder(Telemetry telemetry, ShoulderController shoulderController, LinearOpMode opMode) {
        this.telemetry = telemetry;
        this.shoulderController = shoulderController;
        this.opMode = opMode;
        this.init();
    }

    private void init(){    }

    public Status perform(BlackBoardSingleton blackBoard) {
        Status status;

        if (lastStatus == Status.SUCCESS) {
            return lastStatus;
        }
        if (!started) {
            shoulderController.reset();
            started = true;
            status = Status.RUNNING;
        } else {
            if (shoulderController.isBusy()) {
                status = Status.RUNNING;
            } else {
                if (shoulderController.isShoulderStuck()) {
                    status = Status.FAILURE;
                } else {
                    status = Status.SUCCESS;
                }
            }
        }

        return status;
    }

}
