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
            arm.setTargetPosition(10, 90);
        } else if (gamepad2.dpad_left) {
            arm.setTargetPosition(20, 90);
        } else if (gamepad2.dpad_down) {
            arm.setTargetPosition(0,0);
        }

        if (gamepad1.dpad_down) {
            arm.setTargetPosition(HOME_EXTENSION_INCHES, HOME_ROTATION_DEGREES);
        } else if (gamepad1.dpad_left) {
//            if ((arm.rotationTargetDegrees() == LOW_BAR_ROTATION_DEGREES
//                    && arm.extensionTargetInches() == LOW_BAR_EXTENSION_INCHES)
//                    || (arm.rotationTargetDegrees() == HIGH_BAR_ROTATION_DEGREES
//                        && arm.extensionTargetInches() == HIGH_BAR_EXTENSION_INCHES)
//            ) {
//                arm.hookOnBar(WALL_EXTENSION_INCHES, WALL_ROTATION_DEGREES);
//            }
        } else if (gamepad1.dpad_right) {
//            if ((arm.rotationTargetDegrees() == LOW_BAR_ROTATION_DEGREES
//                    && arm.extensionTargetInches() == LOW_BAR_EXTENSION_INCHES)
//                    || (arm.rotationTargetDegrees() == HIGH_BAR_ROTATION_DEGREES
//                        && arm.extensionTargetInches() == HIGH_BAR_EXTENSION_INCHES)
//            ) {
//                arm.hookOnBar(GROUND_ROTATION_DEGREES, GROUND_ROTATION_DEGREES);
//            }
        } else if (gamepad1.triangle) {
            arm.setTargetPosition(HIGH_BAR_EXTENSION_INCHES, HIGH_BAR_ROTATION_DEGREES);
        } else if (gamepad1.square) {
            arm.setTargetPosition(LOW_BAR_EXTENSION_INCHES, LOW_BAR_ROTATION_DEGREES);
        } else if (gamepad1.circle) {
            arm.setTargetPosition(HIGH_BASKET_EXTENSION_INCHES, HIGH_BASKET_ROTATION_DEGREES);
        } else if (gamepad1.cross) {
            arm.setTargetPosition(LOW_BASKET_EXTENSION_INCHES, LOW_BASKET_ROTATION_DEGREES);
        } else {
            // TODO Implement manual control
        }
        arm.update();
        arm.debugGlobal();
        arm.debugPosition();
    }
}
