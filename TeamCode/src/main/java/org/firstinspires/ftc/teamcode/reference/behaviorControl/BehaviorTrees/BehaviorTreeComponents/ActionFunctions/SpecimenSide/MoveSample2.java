package org.firstinspires.ftc.teamcode.reference.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.SpecimenSide;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.driveTrain.DriveTrainController;

public class MoveSample2 implements ActionFunction {
    Telemetry telemetry;
    DriveTrainController driveTrainController;

    protected Status lastStatus = Status.FAILURE;

    /// TODO: set the right poses
    private final Pose scorePose = new Pose(28, 63, Math.toRadians(0));
    private final Pose ctrlPt1 = new Pose(30.3, 42.6, Math.toRadians(0));
    private final Pose ctrlPt2 = new Pose(64.5, 37.7, Math.toRadians(0));
    private final Pose moveSample2Pose = new Pose(61, 24, Math.toRadians(0));
    private final Pose moveSample2StagingPose = new Pose(17, 16.5, Math.toRadians(0));
    private final Pose ctrlPt3 = new Pose(31.7, 24.3, Math.toRadians(0));


    PathChain moveSample2;
    boolean started = false;


    public MoveSample2(Telemetry telemetry, DriveTrainController driveTrainController) {
        this.telemetry = telemetry;
        this.driveTrainController = driveTrainController;
        this.init();
    }

    private void init() {
        moveSample2 = driveTrainController.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(scorePose),
                                new Point(ctrlPt1),
                                new Point(ctrlPt2),
                                new Point(moveSample2Pose)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(
                        new BezierCurve(
                                new Point(moveSample2Pose),
                                new Point(ctrlPt3),
                                new Point(moveSample2StagingPose)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

    }

    public Status perform(BlackBoard blackBoard) {
        Status status;

        if (lastStatus == Status.SUCCESS) {
            return lastStatus;
        }
        /// TODO: put code here to move the robot from current position to push sample2 in staging area
        if (!started) {
            driveTrainController.followPath(moveSample2, true);
            started = true;
            status = Status.RUNNING;
        } else {
            if (driveTrainController.isBusy()) {
                status = Status.RUNNING;
            } else {
                if (driveTrainController.isRobotStuck()) {
                    status = Status.FAILURE;
                } else {
                    status = Status.SUCCESS;
                }
            }
        }

        driveTrainController.update();
        lastStatus = status;
        return status;
    }

}

