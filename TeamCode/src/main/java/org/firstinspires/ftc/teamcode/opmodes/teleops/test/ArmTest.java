package org.firstinspires.ftc.teamcode.opmodes.teleops.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;

@TeleOp(name = "Test - Arm", group = "Test")
public class ArmTest extends OpMode {
    private Arm arm;

    private final double HOME_EXTENSION_INCHES = 0;
    private final double HOME_ROTATION_DEGREES = 45;

    private final double GROUND_EXTENSION_INCHES = 0;
    private final double GROUND_ROTATION_DEGREES = 6;

    private final double LOW_BAR_EXTENSION_INCHES = 8;
    private final double LOW_BAR_ROTATION_DEGREES = 41;

    private final double HIGH_BAR_EXTENSION_INCHES = 13.5;
    private final double HIGH_BAR_ROTATION_DEGREES = 74;

    private final double LOW_BASKET_EXTENSION_INCHES = 21;
    private final double LOW_BASKET_ROTATION_DEGREES = 61;

    private final double HIGH_BASKET_EXTENSION_INCHES = 38;
    private final double HIGH_BASKET_ROTATION_DEGREES = 72;

    private final double WALL_EXTENSION_INCHES = 0;
    private final double WALL_ROTATION_DEGREES = 37;

    private final double CLEAR_SUB_BAR_EXTENSION_INCHES = 0;
    private final double CLEAR_SUB_BAR_ROTATION_DEGREES = 17;

    private final double JUST_OVER_BLOCK_EXTENSION_INCHES = 0;
    private final double JUST_OVER_BLOCK_ROTATION_DEGREES = 12;

    @Override public void init() {
       arm = new Arm(this);
    }

    @Override public void loop() {
        if (gamepad1.left_bumper) {
            arm.openIntake();
        } else if (gamepad1.right_bumper) {
            arm.closeIntake();
        }

        if (gamepad2.dpad_up) {
            arm.setTargetPosition(0, 90);
        } else if (gamepad2.dpad_left) {
            arm.setTargetPosition(40, 90);
        } else if (gamepad2.dpad_down) {
            arm.setTargetPosition(0, 0);
        }

        arm.update();
        arm.debugGlobal();
        arm.debugPosition();
    }
}
