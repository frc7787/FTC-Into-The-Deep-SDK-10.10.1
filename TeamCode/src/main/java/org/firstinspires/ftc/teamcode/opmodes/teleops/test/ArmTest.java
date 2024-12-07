package org.firstinspires.ftc.teamcode.opmodes.teleops.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;

@TeleOp
public class ArmTest extends OpMode {
    private Arm arm;

    private boolean shouldUpdate = true;

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

    private Gamepad previousGamepad, currentGamepad;

    @Override public void init() {
        arm = new Arm(this);
        previousGamepad = new Gamepad();
        currentGamepad = new Gamepad();
    }

    @Override public void loop() {
        previousGamepad.copy(currentGamepad);
        currentGamepad.copy(gamepad1);

        if (shouldUpdate) {
            arm.update();
        } else {
            arm.debugSetPowers(0, gamepad1.left_stick_y * -1.0);
        }

        if (currentGamepad.cross && !previousGamepad.cross) shouldUpdate = !shouldUpdate;

        telemetry.addData("Should Update", shouldUpdate);

        if (gamepad1.dpad_down) {
            arm.setTargetPosition(0,0);
        } else if (gamepad1.dpad_up) {
            arm.setTargetPosition(40, 40);
        } else if (gamepad1.dpad_right) {
            arm.setTargetPosition(30, 30);
        } else if (gamepad1.dpad_left) {
            arm.setTargetPosition(20, 20);
        }

        arm.debugPosition();
    }
}
