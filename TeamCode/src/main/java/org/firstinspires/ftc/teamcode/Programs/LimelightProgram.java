package org.firstinspires.ftc.teamcode.Programs;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

import dev.nextftc.ftc.NextFTCOpMode;

@TeleOp(name = "NextFTC TeleOp Program Java")
public class LimelightProgram extends NextFTCOpMode {
    Limelight3A limelight;
    LLResult result = limelight.getLatestResult();
    FtcDashboard dashboard = FtcDashboard.getInstance();


    public LimelightProgram() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        limelight.pipelineSwitch(1); // Switch to pipeline number 0
        limelight.start();
        FtcDashboard.getInstance().startCameraStream(limelight, 0);
    }
    @Override
    public void onStartButtonPressed() {
        result.getPipelineIndex();
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fr : fiducialResults) {
            telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
        }
    }
}