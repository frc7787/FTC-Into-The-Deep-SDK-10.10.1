package org.firstinspires.ftc.teamcode.opmodes.teleops.test;

import static com.qualcomm.robotcore.hardware.Servo.Direction.*;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp
public final class ServoTest extends OpMode {
    private static ServoImplEx testServo;

    public static String SERVO_NAME     = "testServo";
    public static double START_POSITION = 0.0;
    public static Servo.Direction SERVO_DIRECTION = FORWARD;

    private Gamepad previousGamepad, currentGamepad;

    private double servoPosition;

    @Override public void init() {
        testServo = hardwareMap.get(ServoImplEx.class, SERVO_NAME);
        testServo.setPwmEnable();
        testServo.setDirection(SERVO_DIRECTION);
        testServo.setPosition(START_POSITION);
        servoPosition = START_POSITION;
        previousGamepad = new Gamepad();
        currentGamepad  = new Gamepad();
    }

    @Override public void init_loop() {
        previousGamepad.copy(currentGamepad);
        currentGamepad.copy(gamepad1);

        telemetry.addLine("Press Options (The One On The Right) To Display Controls.");
        if (gamepad1.options) displayInstructions();
        telemetry.addData("Start Position", servoPosition);

        listenForServoPositionChange();

        telemetry.update();
    }

    @Override public void loop() {
        previousGamepad.copy(currentGamepad);
        currentGamepad.copy(gamepad1);

        telemetry.addLine("Press Options (The One On The Left) To Display Controls.");

        if (gamepad1.options) {
            displayInstructions();
        } else {
            displayServoTelemetry();
        }

        listenForServoPositionChange();
        listenForServoConfigurationChange();

        testServo.setPosition(servoPosition);
    }

    private void listenForServoConfigurationChange() {
        if (currentGamepad.cross && !previousGamepad.cross) {
            switch (testServo.getDirection()) {
                case FORWARD:
                    testServo.setDirection(REVERSE);
                    break;
                case REVERSE:
                    testServo.setDirection(FORWARD);
                    break;
            }
        }

        if (currentGamepad.circle && !previousGamepad.circle) {
            if (testServo.isPwmEnabled()) {
                testServo.setPwmDisable();
            } else {
                testServo.setPwmEnable();
            }
        }
    }

    private void listenForServoPositionChange() {
        if (currentGamepad.dpad_up && !previousGamepad.dpad_up) {
            servoPosition += 0.01;
        }

        if (currentGamepad.dpad_down && !previousGamepad.dpad_down) {
            servoPosition -= 0.01;
        }

        if (servoPosition < 0) servoPosition = 0;
    }

    private void displayInstructions() {
        telemetry.addLine("Press X To Invert The Direction");
        telemetry.addLine("Press O To Toggle The PWM");
        telemetry.addLine("Press Dpad Up/Down To Increment/Decrement The Position");
    }

    private void displayServoTelemetry() {
        telemetry.addData("Direction", testServo.getDirection());
        telemetry.addData("PWM Enabled", testServo.isPwmEnabled());
        telemetry.addData("Position", testServo.getPosition());
    }
}