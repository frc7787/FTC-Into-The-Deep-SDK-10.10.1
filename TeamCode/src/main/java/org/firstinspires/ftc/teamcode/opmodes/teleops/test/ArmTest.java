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

        if (currentGamepad.dpad_up && !previousGamepad.dpad_up) {
            arm.setTargetPosition(0, 500);
        } else if (currentGamepad.dpad_left && !previousGamepad.dpad_left) {
            arm.setTargetPosition(3000, 0);
        } else if (currentGamepad.dpad_right && !previousGamepad.dpad_right) {
            arm.setTargetPosition(-3000, 0);
        } else {
            double leftStickY  = -gamepad2.left_stick_y;
            double rightStickY = -gamepad2.right_stick_y;

            if (Math.abs(leftStickY) > 0.05) {
                int rotationTargetPosition = arm.rotationTargetPosition();
                rotationTargetPosition += (int) (leftStickY * 100);
                arm.setRotationTargetPosition(rotationTargetPosition);
            }

            if (Math.abs(rightStickY) > 0.05) {
                int extensionTargetPosition = arm.extensionTargetPosition();
                extensionTargetPosition += (int) (rightStickY * 30);
                arm.setExtensionTargetPosition(extensionTargetPosition);
            }
        }

        arm.debugAll();
    }
}
