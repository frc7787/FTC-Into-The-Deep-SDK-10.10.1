package org.firstinspires.ftc.teamcode.opmodes.teleops.test;

import com.qualcomm.robotcore.eventloop.opmode.*;
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;

@TeleOp(name = "Test - Arm", group = "Test")
public class ArmTest extends OpMode {
    private Arm arm;

    @Override public void init() {
        arm = new Arm(this);
    }

    @Override public void loop() {
        arm.update();

        if (gamepad1.dpad_up) {
            arm.setTargetPosition(0, 500);
        } else if (gamepad1.dpad_left) {
            arm.setTargetPosition(500, 0);
        } else if (gamepad1.dpad_right) {
            arm.setTargetPosition(-500, 0);
        } else if (gamepad1.dpad_down) {
            arm.setTargetPosition(0,0);
        }
    }
}
