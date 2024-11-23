package org.firstinspires.ftc.teamcode.opmodes.teleops.test;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;

@TeleOp(name = "Test - Arm", group = "Test")
public final class ArmTest extends OpMode {
    private Arm arm;
    private Gamepad currentGamepad, previousGamepad;

    @Override public void init() {
        arm = new Arm(this);
        currentGamepad  = new Gamepad();
        previousGamepad = new Gamepad();
    }

    @Override public void loop() {
        previousGamepad.copy(currentGamepad);
        currentGamepad.copy(gamepad2);

        arm.update();

        if (gamepad1.left_bumper) {
            arm.setTargetPosition(0, 600);
        } else if (gamepad1.right_bumper) {
            arm.setTargetPosition(0,0);
        }

        arm.debugGlobal();
        arm.debugExtension();
    }
}
