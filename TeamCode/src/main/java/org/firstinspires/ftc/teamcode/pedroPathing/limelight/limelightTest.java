package org.firstinspires.ftc.teamcode.pedroPathing.limelight;

import android.graphics.Color;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "LimelightTest", group = "testing")
public class limelightTest extends OpMode {
    private Limelight3A limelight;

    @Override
    public void init(){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

    }
    @Override
    public void start(){
        limelight.start();
    }
    @Override
    public void stop(){
        limelight.stop();
    }

    @Override
    public void loop(){
        LLResult result = limelight.getLatestResult();
        if(result != null && result.isValid()){
            telemetry.addData("result: ", result);
        }
    }


}

