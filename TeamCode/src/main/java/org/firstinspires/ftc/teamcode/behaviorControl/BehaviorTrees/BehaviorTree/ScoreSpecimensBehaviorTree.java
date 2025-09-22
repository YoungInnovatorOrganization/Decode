package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTree;

import com.pedropathing.localization.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;
import java.util.List;

import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.ActionFunctions.SpecimenSide.MoveArmToScoringSpecimenPosition;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Action;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BehaviorTree;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoard;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Node;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Sequence;
import org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.arm.ArmController;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.claw.ClawController;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.driveTrain.DriveTrainController;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.shoulder.ShoulderController;


public class ScoreSpecimensBehaviorTree {
    private BehaviorTree tree;
    private Node root;
    private BlackBoard blackBoard;
    protected Telemetry telemetry;
    protected HardwareMap hardwareMap;
    protected LinearOpMode opMode;

    /// Drivetrain
    protected DriveTrainController driveTrainController;
    /// TODO: set the startingPose to the right value
    private final Pose startingPose = new Pose(10, 55, Math.toRadians(0));
    /// Emd Drivetrain

    /// Shoulder
    protected ShoulderController shoulderController;

    /// End Shoulder

    /// Arm
    protected ArmController armController;

    /// End Arm

    /// Claw
    protected ClawController clawController;

    /// End Claw
    public ScoreSpecimensBehaviorTree( LinearOpMode opMode) {
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
        this.opMode = opMode;

        Init();
    }
    private void Init() {
        this.blackBoard = BlackBoard.getInstance(telemetry);
        this.blackBoard.reset();
/*
        /// Drive Train
        this.driveTrainController = DriveTrainController.getInstance();
        this.driveTrainController.reset();
        this.driveTrainController.initialize(hardwareMap, startingPose);
        /// End Drivetrain
*/
        /// Shoulder
        this.shoulderController = ShoulderController.getInstance();

        this.shoulderController.reset();
        this.shoulderController.initialize(hardwareMap, telemetry, this.opMode);

        /// End Shoulder

        /// Arm
        this.armController = ArmController.getInstance();

        this.armController.reset();
        this.armController.initialize(hardwareMap, telemetry, this.opMode);

        /// End Arm

        /// Claw
        this.clawController = ClawController.getInstance();
        this.clawController.reset();
        this.clawController.initialize(0,this.opMode);
        /// End Claw

        telemetry.clearAll();

        this.root = new Sequence(
                Arrays.asList(
                        //new Action(new DriveTrainControllerUpdate(telemetry, this.driveTrainController),telemetry),
                        //new Action(new ScoreSpecimen1(telemetry, this.driveTrainController),telemetry),
                        //new Action(new MoveSample2(telemetry, this.driveTrainController),telemetry),
                        //new Action(new MoveSample3(telemetry, this.driveTrainController), telemetry),
                        //new Action(new ScoreSpecimen2(telemetry, driveTrainController), telemetry),
                       // new Action(new MoveShoulderToScoringPosition(telemetry, shoulderController, this.opMode), telemetry),
                        //new Action(new MoveShoulderToScoreSpecimen(telemetry, shoulderController, this.opMode), telemetry),
                       // new Action(new MoveShoulderToDefaultPosition(telemetry, shoulderController, this.opMode), telemetry)
                        new Action(new MoveArmToScoringSpecimenPosition(telemetry, armController, this.opMode), telemetry)
                        //new Action(new OpenClaw(telemetry, clawController, this.opMode), telemetry),
                        //new Action(new PauseAction(2000,telemetry, this.opMode), telemetry),
                        //new Action(new CloseClaw(telemetry, clawController, this.opMode), telemetry),
                        //new Action(new PauseAction(2000,telemetry, this.opMode), telemetry),
                        //new Action(new OpenClaw(telemetry, clawController, this.opMode), telemetry),
                        //new Action(new PauseAction(2000,telemetry, this.opMode), telemetry),
                        //new Action(new CloseClaw(telemetry, clawController, this.opMode), telemetry)


                ),telemetry);

        this.tree = new BehaviorTree(root, blackBoard);
    }

    public Status tick() {
        // Clear the bulk cache for each Lynx module hub. This must be performed once per loop
        // as the bulk read caches are being handled manually.
        List<LynxModule> allHubs;

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        // Run the behavior tree
        Status result = tree.tick();
        telemetry.addData("ScoreSpecimensBehaviorTree", "Run - Behavior tree result: %s",result);
        telemetry.update();

        return result;
    }
}

