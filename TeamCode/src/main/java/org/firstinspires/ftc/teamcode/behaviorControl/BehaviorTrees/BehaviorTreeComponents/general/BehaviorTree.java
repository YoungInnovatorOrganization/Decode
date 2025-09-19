package org.firstinspires.ftc.teamcode.behaviorControl.BehaviorTrees.BehaviorTreeComponents.general;


public class BehaviorTree {
    private final Node root;
    private final behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoardSingleton blackBoard;

    public BehaviorTree(Node root, behaviorControl.BehaviorTrees.BehaviorTreeComponents.general.BlackBoardSingleton blackBoard) {
        this.root = root;
        this.blackBoard = blackBoard;
    }

    public Status tick() {
        return root.execute(blackBoard);
    }
}
