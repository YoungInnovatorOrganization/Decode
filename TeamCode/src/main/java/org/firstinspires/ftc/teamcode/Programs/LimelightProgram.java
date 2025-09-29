package org.firstinspires.ftc.teamcode.Programs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
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
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.internal.network.ControlHubDeviceNameManager;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp(name = "LimelightProgram")
public class LimelightProgram extends NextFTCOpMode {
    MotorEx Direita = new MotorEx("motor_shooter");
    
    MotorEx DireitaTras = new MotorEx("motor_shooter2");

    MotorEx Esquerda = new MotorEx("motor_esquerda");
    MotorEx EsquerdaTras = new MotorEx("motor_esquerdatras");

    double Heading;
    double Strafe;
    double Drive;
    Limelight3A limelight;


    //FtcDashboard dashboard = FtcDashboard.getInstance();


    @Override
    public void onInit() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.pipelineSwitch(1); // Switch to pipeline number 0
        limelight.start();
        FtcDashboard.getInstance().startCameraStream(limelight, 90);

        //FtcDashboard.getInstance().startCameraStream(limelight, 0);
    }

    @Override
    public void onWaitForStart(){
        FtcDashboard.getInstance().startCameraStream(limelight, 90);
    }
    @Override
    public void onUpdate() {
        TelemetryPacket packet = new TelemetryPacket();
        Telemetry telemetry1;
        LLStatus status = limelight.getStatus();
        telemetry.addData("Name", "%s",
                status.getName());
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(),(int)status.getFps());
        telemetry.addData("Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());
        telemetry.update();
        LLResult result = limelight.getLatestResult();
        result.getPipelineIndex();
        Pose3D botpose = result.getBotpose();

        if (result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                Heading = (-fr.getTargetXDegrees()/22);

            }
        }else{
            telemetry.addData("Limelight", "No data available");
        }
        Strafe = -gamepad1.left_stick_x;
        Drive = gamepad1.left_stick_y;
        new MotorEx(Esquerda.getMotor())
                .setPower(Heading+(Strafe/2)+(Drive/2));
        new MotorEx(Direita.getMotor())
                .setPower(Heading+(Strafe/2)-(Drive/2));
        new MotorEx(DireitaTras.getMotor())
                .setPower(-(Strafe/2)-(Drive/2));
        new MotorEx(EsquerdaTras.getMotor())
                .setPower(-(Strafe/2)+(Drive/2));
        telemetry.addData("Strafe X", Strafe);
        telemetry.addData("Drive Y", Drive);
        telemetry.addData("Heading LL", Heading);
        telemetry.addData("Botpose", botpose.toString());
        packet.put("Botpose", botpose.toString());
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.sendTelemetryPacket(packet);


        //pose3D botpose = result.getBotPose();


    }

}