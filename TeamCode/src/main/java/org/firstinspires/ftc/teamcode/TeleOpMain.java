package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.subsystems.mecanumdrive.DriveMode.FIELD_CENTRIC;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.mecanumdrive.MecanumDrive;

@TeleOp(name = "TeleOp - Use This One", group = "$")
public class TeleOpMain extends OpMode {
    // ---------------------------------------------------------------------------------------------
    // Hardware
    // ---------------------------------------------------------------------------------------------

    private Arm arm;
    private Servo clawServo;
    private MecanumDrive driveBase;

    // ---------------------------------------------------------------------------------------------
    // Position Constants
    // ---------------------------------------------------------------------------------------------

    private double CLAW_CLOSED_POSITION = 0.00;
    private double CLAW_OPEN_POSITION   = 0.02;

    private final double GROUND_EXTENSION_INCHES = 0;
    private final double GROUND_ROTATION_DEGREES = 6;

    private final double LOW_BAR_EXTENSION_INCHES = 8;
    private final double LOW_BAR_ROTATION_DEGREES = 41;

    private final double HIGH_BAR_EXTENSION_INCHES = 13.5;
    private final double HIGH_BAR_ROTATION_DEGREES = 74;

    private final double LOW_BASKET_EXTENSION_INCHES = 21;
    private final double LOW_BASKET_ROTATION_DEGREES = 61;

    private final double HIGH_BASKET_EXTENSION_INCHES = 39;
    private final double HIGH_BASKET_ROTATION_DEGREES = 72;

    private final double WALL_EXTENSION_INCHES = 0;
    private final double WALL_ROTATION_DEGREES = 37;

    private final double CLEAR_SUB_BAR_EXTENSION_INCHES = 0;
    private final double CLEAR_SUB_BAR_ROTATION_DEGREES = 17;

    private final double JUST_OVER_BLOCK_EXTENSION_INCHES = 0;
    private final double JUST_OVER_BLOCK_ROTATION_DEGREES = 12;

    @Override public void init() {
        driveBase = new MecanumDrive(this, FIELD_CENTRIC);
        arm       = new Arm(this);
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        clawServo.setPosition(CLAW_CLOSED_POSITION);
    }

    @Override public void loop() {
        // --------------------
        // Drivebase
        // --------------------

        driveBase.drive(
                gamepad1.left_stick_y * -1.0,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x
        );

        if (gamepad1.options) driveBase.resetYaw();

        // --------------------
        // Intake
        // --------------------

        if (gamepad2.left_bumper) {
            clawServo.setPosition(CLAW_OPEN_POSITION);
        } else if (gamepad2.right_bumper) {
            clawServo.setPosition(CLAW_CLOSED_POSITION);
        }

        // --------------------
        // Arm
        // --------------------

        double extensionTargetInches = arm.targetExtensionInches();
        double rotationTargetDegrees = arm.targetAngleDegrees();

        if (gamepad2.dpad_down) {
            extensionTargetInches = GROUND_EXTENSION_INCHES;
            rotationTargetDegrees = GROUND_ROTATION_DEGREES;
        } else if (gamepad2.dpad_left) {
            extensionTargetInches = LOW_BASKET_EXTENSION_INCHES;
            rotationTargetDegrees = LOW_BASKET_ROTATION_DEGREES;
        } else if (gamepad2.dpad_up) {
            extensionTargetInches = HIGH_BASKET_EXTENSION_INCHES;
            rotationTargetDegrees = HIGH_BASKET_ROTATION_DEGREES;
        } else if (gamepad2.cross) {
            extensionTargetInches = LOW_BAR_EXTENSION_INCHES;
            rotationTargetDegrees = LOW_BAR_ROTATION_DEGREES;
        } else if (gamepad2.square) {
            extensionTargetInches = HIGH_BAR_EXTENSION_INCHES;
            rotationTargetDegrees = HIGH_BAR_ROTATION_DEGREES;
        } else {
            double rotationPower = gamepad2.right_stick_y * -1.0;
            double extensionPower = gamepad2.left_stick_y * -1.0;

            if (Math.abs(extensionPower) >= 0.05) {
               extensionTargetInches += rotationPower * 0.3;
            }

            if (Math.abs(rotationPower) >= 0.05) {
                rotationTargetDegrees += extensionPower * 0.3;
            }
        }

        arm.setTargetPosition(extensionTargetInches, rotationTargetDegrees);
        arm.update();
        arm.debugGlobal();
        arm.debugPosition();
    }
}
