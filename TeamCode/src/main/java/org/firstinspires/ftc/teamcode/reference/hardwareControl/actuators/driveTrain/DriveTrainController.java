package org.firstinspires.ftc.teamcode.reference.hardwareControl.actuators.driveTrain;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;


import com.pedropathing.follower.Follower;


import org.firstinspires.ftc.teamcode.hardwareConfig.actuators.driveTrain.FConstants;
import org.firstinspires.ftc.teamcode.hardwareConfig.actuators.driveTrain.LConstants;

public class DriveTrainController {
    private Follower follower;
    private boolean initialized = false;

    // Private static instance (eager initialization)
    private static final DriveTrainController INSTANCE = new DriveTrainController();

    // Private constructor to prevent instantiation
    private DriveTrainController() {
        // Initialize hardware, state, or configuration here

    }
    // Initialization method — must be called once at the beginning
    public void initialize(HardwareMap hardwareMap, Pose startPose) {
        if (initialized) {
            return;
            //throw new IllegalStateException("DriveTrainController has already been initialized.");
        }

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        initialized = true;
    }

    public void reset() {
        if(initialized) {
            follower.breakFollowing();
            follower = null;
            initialized = false;
        }
    }

    // Public method to access the singleton instance
    public static DriveTrainController getInstance() {
        return INSTANCE;
    }

    // Example method
    public void followPath(PathChain pathChain, boolean holdEnd) {
        if(!follower.isBusy()) {
            follower.followPath(pathChain,holdEnd);
        }
    }

    public void followPath(Path path, boolean holdEnd) {
        if(!follower.isBusy()) {
            follower.followPath(path,holdEnd);
        }
    }

    public boolean isBusy(){
        return follower.isBusy();
    }

    public void update(){
        follower.update();
    }
    public PathBuilder pathBuilder() {
        return follower.pathBuilder();
    }
    public boolean isRobotStuck(){
        return follower.isRobotStuck();
    }
    public void breakFollowing() {
         follower.breakFollowing();
    }
}

/*usage Example
*  Pose startingPose = new Pose(0, 0, 0);  // Change if needed

        // Initialize singleton
        DriveTrainController drive = DriveTrainController.getInstance();
        drive.initialize(hardwareMap, startingPose); //hardwareMap is from opMode
*
* */
