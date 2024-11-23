package org.firstinspires.ftc.teamcode.opmodes.teleops;

import static org.firstinspires.ftc.teamcode.subsystems.mecanumdrive.MecanumDriveBase.DriveMode.FIELD_CENTRIC;
import static org.firstinspires.ftc.teamcode.subsystems.mecanumdrive.MecanumDriveBase.DriveMode.ROBOT_CENTRIC;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.mecanumdrive.MecanumDriveBase;

@TeleOp(name = "TeleOp - Use This One", group = "a")
public class TeleOpMain extends OpMode {
    private Arm arm;
    private MecanumDriveBase driveBase;
    private Servo clawServo;

    private static final int LOW_BUCKET_ROTATION  = 3065;
    private static final int LOW_BUCKET_EXTENSION = 926;

    private static final int HIGH_BUCKET_ROTATION  = 3450;
    private static final int HIGH_BUCKET_EXTENSION = 1740;

    private static final int LOW_BAR_ROTATION  = 2050;
    private static final int LOW_BAR_EXTENSION = 440;

    private static final int HIGH_BAR_ROTATION  = 3044;
    private static final int HIGH_BAR_EXTENSION = 970;


    @Override public void init() {
       arm       = new Arm(this);
       driveBase = new MecanumDriveBase(this, ROBOT_CENTRIC);
       clawServo = hardwareMap.get(Servo.class, "clawServo");
    }

    @Override public void loop() {
        arm.update();

        double leftStickY  = gamepad1.left_stick_y * -1.0;
        double leftStickX  = gamepad1.left_stick_x;
        double rightStickX = gamepad1.right_stick_x;

        driveBase.drive(
                Math.abs(leftStickY)  * leftStickY,
                Math.abs(leftStickX)  * leftStickX,
                Math.abs(rightStickX) * rightStickX
        );

        if (gamepad2.left_bumper) {
            clawServo.setPosition(0.02);
        }

        if (gamepad2.right_bumper) {
            clawServo.setPosition(0.0);
        }

        if (gamepad2.dpad_down) {
            arm.setTargetPosition(0,0);
        } else if (gamepad2.dpad_up) {
            arm.setTargetPosition(HIGH_BAR_ROTATION, HIGH_BAR_EXTENSION);
        } else if (gamepad2.dpad_left) {
            arm.setTargetPosition(LOW_BAR_EXTENSION, LOW_BAR_ROTATION);
        } else if (gamepad2.triangle) {
            arm.setTargetPosition(HIGH_BUCKET_ROTATION, HIGH_BUCKET_EXTENSION);
        } else if (gamepad2.square) {
            arm.setTargetPosition(LOW_BUCKET_ROTATION, LOW_BUCKET_EXTENSION);
        } else {
            double gamepad2LeftStickY  = gamepad2.right_stick_y * -1.0;
            double gamepad2RightStickY = gamepad2.left_stick_y * -1.0;

            if (Math.abs(gamepad2LeftStickY) >= 0.05) {
                int rotationTargetPosition = arm.rotationTargetPosition();
                rotationTargetPosition += (int) (gamepad2LeftStickY * 100);
                arm.setRotationTargetPosition(rotationTargetPosition);
            }

            if (Math.abs(gamepad2RightStickY) >= 0.05) {
                int extensionTargetPosition = arm.extensionTargetPosition();;
                extensionTargetPosition += (int) (gamepad2RightStickY * 50);
                arm.setExtensionTargetPosition(extensionTargetPosition);
            }
        }

        arm.debugAll();
    }
}
