package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.SpecimenSide;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.driveTrain.DriveTrainController;

public class ScoreSpecimen2 implements ActionFunction
{
    Telemetry telemetry;
    DriveTrainController driveTrainController;
    protected Status lastStatus = Status.FAILURE;

    /// TODO: set the right poses
    private final Pose moveSample3StagingPose = new Pose(17, 13, Math.toRadians(0));
    private final Pose moveSpecimen2GrabPose = new Pose(20, 30, Math.toRadians(180));
    private final Pose ctrlPt1 = new Pose(39, 16, Math.toRadians(0));
    private final Pose scorePose = new Pose(28, 63, Math.toRadians(0));
    PathChain scoreSpecimen2;
    boolean started = false;

    public ScoreSpecimen2(Telemetry telemetry, DriveTrainController driveTrainController) {
        this.telemetry = telemetry;
        this.driveTrainController = driveTrainController;
        this.init();
    }

    private void init(){
        scoreSpecimen2 = driveTrainController.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(moveSample3StagingPose),
                        new Point(ctrlPt1),
                        new Point(moveSpecimen2GrabPose)))
                .setLinearHeadingInterpolation(moveSample3StagingPose.getHeading(), moveSpecimen2GrabPose.getHeading())
                .addPath(new BezierLine(
                        new Point(moveSpecimen2GrabPose),
                        new Point(scorePose)))
                .setLinearHeadingInterpolation(moveSpecimen2GrabPose.getHeading(), scorePose.getHeading())
                .build();
    }

    public Status perform(BlackBoard blackBoard) {
        Status status;

        if (lastStatus == Status.SUCCESS) {
            return lastStatus;
        }
        if(!started){
            driveTrainController.followPath(scoreSpecimen2,true);
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
