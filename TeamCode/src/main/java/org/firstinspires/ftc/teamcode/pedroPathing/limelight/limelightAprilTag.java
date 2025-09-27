package org.firstinspires.ftc.teamcode.pedroPathing.limelight;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;

import java.util.ArrayList;
import java.util.List;

/**
 * A sample Limelight class for FTC. This class handles Limelight initialization
 * and provides methods to retrieve raw AprilTag detection data.
 * It is designed for tasks where robot localization is not required.
 */
public class limelightAprilTag {

    private Limelight3A limelight;



    /**
     * Initializes the Limelight hardware.
     * @param hardwareMap The hardware map of the robot.
     * @param limelightName The name of the Limelight in the hardware map.
     */
    public void LimelightAprilTag(HardwareMap hardwareMap, String limelightName) {
        this.limelight = hardwareMap.get(Limelight3A.class, limelightName);

        // Make sure the Limelight is using the correct pipeline

        limelight.pipelineSwitch(0);
    }

    /**
     * Get the latest result from the Limelight. This is the primary
     * method for getting data.
     * @return The latest LLResult from the Limelight.
     */
    public LLResult getLatestResult() {
        return limelight.getLatestResult();
    }

    /**
     * Get a list of all detected AprilTags.
     * @return A list of FiducialResult objects.
     */
    public List<FiducialResult> getDetectedTags() {
        LLResult result = getLatestResult();
        if (result != null && result.isValid()) {
            return result.getFiducialResults();
        }
        return new ArrayList<>();
    }


}
