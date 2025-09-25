package org.firstinspires.ftc.teamcode.Programs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

import dev.nextftc.ftc.NextFTCOpMode;

@TeleOp(name = "LimelightOnly")
public class LimelightOnly extends NextFTCOpMode {

    Limelight3A limelight;

    double alturaCamera = 0.12;
    double alturaTag = 0.75;

    @Override
    public void onInit() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(1);
        limelight.start();
        FtcDashboard.getInstance().startCameraStream(limelight, 90);
    }

    @Override
    public void onWaitForStart() {
        FtcDashboard.getInstance().startCameraStream(limelight, 90);
    }

    @Override
    public void onUpdate() {
        LLStatus status = limelight.getStatus();
        telemetry.addData("Name", "%s", status.getName());
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(), (int)status.getFps());
        telemetry.addData("Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());

        LLResult result = limelight.getLatestResult();

        if (result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducials) {
                telemetry.addData("Fiducial", "ID: %d, Family: %s",
                        fr.getFiducialId(), fr.getFamily());
                telemetry.addData("X Degrees", "%.2f", fr.getTargetXDegrees());
                telemetry.addData("Y Degrees", "%.2f", fr.getTargetYDegrees());

                double anguloVertical = Math.toRadians(fr.getTargetYDegrees());
                double distancia = (alturaTag - alturaCamera) / Math.tan(anguloVertical);
                telemetry.addData("Distancia", "%.2f m", distancia);
            }
        } else {
            telemetry.addData("Limelight", "No data available");
        }

        telemetry.update();
    }
}