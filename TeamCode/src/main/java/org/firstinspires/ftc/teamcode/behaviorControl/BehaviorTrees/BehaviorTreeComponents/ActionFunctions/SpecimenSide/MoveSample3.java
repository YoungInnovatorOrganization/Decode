package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.SpecimenSide;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.driveTrain.DriveTrainController;

public class MoveSample3 implements ActionFunction {
    Telemetry telemetry;
    DriveTrainController driveTrainController;

    protected Status lastStatus = Status.FAILURE;

    /// TODO: set the right poses
    private final Pose moveSample2StagingPose = new Pose(17, 16.5, Math.toRadians(0));
    private final Point ctrlPt1 =  new Point(46.500, 35.000, Point.CARTESIAN);
    private final Point ctrlPt2 =  new Point(87.000, 28.300, Point.CARTESIAN);
    private final Point ctrlPt3 =  new Point(58.700, 8.800, Point.CARTESIAN);
    private final Pose moveSample3StagingPose = new Pose(17, 11, Math.toRadians(0));


    PathChain moveSample3;
    boolean started = false;


    public MoveSample3(Telemetry telemetry, DriveTrainController driveTrainController) {
        this.telemetry = telemetry;
        this.driveTrainController = driveTrainController;
        this.init();
    }

    private void init() {
        moveSample3 = driveTrainController.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(moveSample2StagingPose),
                                ctrlPt1,
                                ctrlPt2,
                                ctrlPt3,
                                new Point(moveSample3StagingPose)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

    }

    public Status perform(BlackBoard blackBoard) {
        Status status;

        if (lastStatus == Status.SUCCESS) {
            return lastStatus;
        }
        /// TODO: put code here to move the robot from current position to push sample2 in staging area
        if (!started) {
            driveTrainController.followPath(moveSample3, true);
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

