package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem;
import org.firstinspires.ftc.teamcode.utility.MotorUtility;

@TeleOp(name = "Test - Arm", group = "Test")
public class ArmTest extends OpMode {
    private ArmSubsystem armSubsystem;

    @Override public void init() {
        armSubsystem = new ArmSubsystem(this);
    }

    @Override public void loop() {
        armSubsystem.displayCurrent();
        telemetry.update();
    }
}
