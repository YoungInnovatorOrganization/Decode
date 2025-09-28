package org.firstinspires.ftc.teamcode.Autos;

import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

@Config
public class Auto_PPG extends NextFTCOpMode {
    {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Shooter.INSTANCE)
        );
    }
    public static Pose PoseInicial = new Pose(27.468208092485547, 128, Math.toRadians(180));
    @Override public void onInit() { }
    @Override public void onWaitForStart() {

    }
    @Override public void onStartButtonPressed() { }
}
