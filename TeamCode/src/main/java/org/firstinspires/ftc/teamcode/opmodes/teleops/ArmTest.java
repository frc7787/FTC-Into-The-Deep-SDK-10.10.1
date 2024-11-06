package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem;

@TeleOp(name = "Test - Arm", group = "Test")
public class ArmTest extends OpMode {
    private ArmSubsystem armSubsystem;

    @Override public void init() {
        armSubsystem = new ArmSubsystem(this);
    }

    @Override public void loop() {
        armSubsystem.debugCurrent();
        telemetry.update();
    }
}
