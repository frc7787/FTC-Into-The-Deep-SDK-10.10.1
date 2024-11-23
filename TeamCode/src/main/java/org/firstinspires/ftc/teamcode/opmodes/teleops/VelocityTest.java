package org.firstinspires.ftc.teamcode.opmodes.teleops;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.utility.MotorUtility;

@TeleOp(name = "Test - Velocity")
public class VelocityTest extends OpMode {
    private Arm arm;

    @Override public void init() {
        arm = new Arm(this);
    }

    @Override public void loop() {
        arm.debugSetArmPower(gamepad1.left_stick_y, 0);
        arm.debugExtension();
    }
}
