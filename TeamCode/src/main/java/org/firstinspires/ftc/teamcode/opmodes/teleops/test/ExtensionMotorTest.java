package org.firstinspires.ftc.teamcode.opmodes.teleops.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;

@TeleOp
public class ExtensionMotorTest extends OpMode {
    private Arm arm;

    @Override public void init() {
        arm = new Arm(this);
    }

    @Override public void loop() {
        if (gamepad1.dpad_up) {
            arm.setTargetPositionInchesRobotCentric(10, 10);
        } else if (gamepad1.dpad_left) {
            arm.setTargetPositionInchesRobotCentric(-2.0, 7);
        }

        arm.update();
        arm.debugPosition();
    }
}