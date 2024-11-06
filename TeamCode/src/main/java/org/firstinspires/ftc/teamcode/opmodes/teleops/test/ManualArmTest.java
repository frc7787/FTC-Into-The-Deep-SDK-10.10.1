package org.firstinspires.ftc.teamcode.opmodes.teleops.test;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem;

@TeleOp(name = "Test - Manual Arm", group = "Test")
public final class ManualArmTest extends OpMode {
    private ArmSubsystem armSubsystem;

    @Override public void init() {
        armSubsystem = new ArmSubsystem(this);
    }

    @Override public void loop() {
        telemetry.addLine("Control The Rotation With The Left Stick Y");
        telemetry.addLine("Control The Extension With The Right Stick Y");
        armSubsystem.debugSetArmPower(
                gamepad1.left_stick_y  * -1.0,
                gamepad1.right_stick_y * -1.0
        );
        telemetry.addData("Current Position", armSubsystem.extensionCurrentPosition());
    }
}
