package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.SpecimenSide;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoardSingleton;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.driveTrain.DriveTrainController;

public class ScoreSpecimen1 implements ActionFunction
{
    Telemetry telemetry;
    DriveTrainController driveTrainController;
    protected Status lastStatus = Status.FAILURE;

    /// TODO: set the right poses
    private final Pose startPose = new Pose(10, 55, Math.toRadians(0));
    private final Pose scorePose = new Pose(28, 63, Math.toRadians(0));
    PathChain scorePreload;
    boolean started = false;

    public ScoreSpecimen1 (Telemetry telemetry, DriveTrainController driveTrainController) {
        this.telemetry = telemetry;
        this.driveTrainController = driveTrainController;
        this.init();
    }

    private void init(){
        scorePreload = driveTrainController.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
    }

    public Status perform(BlackBoardSingleton blackBoard) {
        Status status;

        if (lastStatus == Status.SUCCESS) {
            return lastStatus;
        }
        if(!started){
            driveTrainController.followPath(scorePreload,true);
            started = true;
            status =Status.RUNNING;
        } else {
            if(driveTrainController.isBusy()) {
                status =Status.RUNNING;
            } else {
                if(driveTrainController.isRobotStuck()){
                    status=Status.FAILURE;
                } else {
                    status=Status.SUCCESS;
                }
            }
        }

        driveTrainController.update();
        lastStatus = status;
        return status;
    }

}
