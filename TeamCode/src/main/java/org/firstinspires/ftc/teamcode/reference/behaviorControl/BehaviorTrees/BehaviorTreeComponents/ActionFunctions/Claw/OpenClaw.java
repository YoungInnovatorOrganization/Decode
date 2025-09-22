package org.firstinspires.ftc.teamcode.reference.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.Claw;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.claw.ClawController;

public class OpenClaw implements ActionFunction {
    private static final double CLAW_OPEN_POSITION = 60;
    Telemetry telemetry;
    protected LinearOpMode opMode;

    ClawController clawController;
    protected Status lastStatus = Status.FAILURE;

    public OpenClaw (Telemetry telemetry, ClawController clawController, LinearOpMode opMode) {
        this.telemetry = telemetry;
        this.opMode = opMode;
        this.clawController = clawController;
        this.init();
    }

    private void init(){


    }
    @Override
    public Status perform(BlackBoard blackBoard) {
        Status status;

        telemetry.addData("OpenClaw", "perform start");
        telemetry.update();

        if (lastStatus == Status.SUCCESS) {
            return lastStatus;
        }
       // opMode.sleep(3000);

        clawController.moveToPosition(CLAW_OPEN_POSITION);
        opMode.sleep(3000);


        status=Status.SUCCESS;

        lastStatus = status;

        telemetry.addData("OpenClaw", "perform finish");
        telemetry.update();
        return status;
    }
}
