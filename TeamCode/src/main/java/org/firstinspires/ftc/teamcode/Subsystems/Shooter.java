package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;

@Config
public class Shooter implements Subsystem {
    public static final Shooter INSTANCE = new Shooter();
    private Shooter() { }
    MotorEx motorShooter = new MotorEx("motor_shooter");
    public static double p = 0;
    public static double i = 0;
    public static double d = 0;

    ControlSystem controlSystem = ControlSystem.builder()
            .velPid(p, i, d)
            .build();

    public Command ON = new RunToVelocity(controlSystem, 0.8).requires(this);
    public Command OFF = new RunToVelocity(controlSystem, 0).requires(this);

    @Override
    public void initialize() {
        // initialization logic (runs on init)
    }

    @Override
    public void periodic() {
        motorShooter.setPower(controlSystem.calculate(motorShooter.getState()));
        // periodic logic (runs every loop)
    }
}