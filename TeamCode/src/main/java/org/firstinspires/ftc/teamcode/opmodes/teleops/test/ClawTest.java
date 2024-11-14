package org.firstinspires.ftc.teamcode.opmodes.teleops.test;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Test - Claw", group = "Test")
public class ClawTest extends OpMode {
    private CRServo testServo;
    private Gamepad currentGamepad, previousGamepad;
    private Servo.Direction servoDirection;

    @Override public void init() {
        testServo       = hardwareMap.get(CRServo.class, "clawServo");
        currentGamepad  = new Gamepad();
        previousGamepad = new Gamepad();
    }

    @Override public void loop() {
        previousGamepad.copy(currentGamepad);
        currentGamepad.copy(gamepad1);

        if (gamepad1.dpad_up) {
            testServo.setPower(0.2);
        } else if (gamepad1.dpad_down){
            testServo.setPower(-0.2);
        } else {
            testServo.setPower(0.0);
        }

        if (currentGamepad.cross && !previousGamepad.cross) {
            if (servoDirection == Servo.Direction.REVERSE) {
               servoDirection = Servo.Direction.FORWARD;
            } else {
                servoDirection = Servo.Direction.REVERSE;
            }
        }
    }
}
