package org.firstinspires.ftc.teamcode.opmodes.teleops.test;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.*;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.*;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.*;
import static org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.*;
import static org.firstinspires.ftc.teamcode.opmodes.teleops.test.MotorTest.ControlMode.*;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.utility.MotorUtility;

@TeleOp(name = "Test - Arm Subsystem", group = "Test")
@Disabled
public final class MotorTest extends OpMode {
    protected enum ControlMode {
        POWER,
        POSITION,
        VELOCITY
    }

    private Gamepad currentGamepad, previousGamepad;

    private int velocityStep = 50;
    private int positionStep = 5;
    private double positionPower = 0.5;

    private ControlMode controlMode;
    private DcMotorImplEx testMotor;

    @Override public void init() {
        currentGamepad = new Gamepad();
        previousGamepad = new Gamepad();

        testMotor = hardwareMap.get(DcMotorImplEx.class, "testMotor");
        MotorUtility.reset(testMotor);
        testMotor.setZeroPowerBehavior(BRAKE);

        controlMode = ControlMode.POWER;
    }

    @Override public void init_loop() {
        previousGamepad.copy(currentGamepad);
        currentGamepad.copy(gamepad1);
    }

    @Override public void loop() {
        previousGamepad.copy(currentGamepad);
        currentGamepad.copy(gamepad1);

        telemetry.addLine("Press options to see controls");
        telemetry.addData("Control Mode", controlMode);

        if (gamepad1.options) {
            displayControls();
        } else {
            displayMotorTelemetry();
        }

        listenForMotorConfigurationChanges();
        controlMotor();
    }

    private void listenForMotorConfigurationChanges() {
        if (currentGamepad.cross && !previousGamepad.cross) {
            testMotor.setDirection(testMotor.getDirection().inverted());
        }

        if (currentGamepad.triangle && !previousGamepad.triangle) {
            switch (testMotor.getZeroPowerBehavior()) {
                case BRAKE:
                    testMotor.setZeroPowerBehavior(FLOAT);
                    break;
                case FLOAT:
                    testMotor.setZeroPowerBehavior(BRAKE);
                    break;
                default:
                    break;
            }
        }

        if (currentGamepad.square && !previousGamepad.square) {
            switch (controlMode) {
                case POWER:
                    controlMode = POSITION;
                    break;
                case POSITION:
                    controlMode = VELOCITY;
                    break;
                case VELOCITY:
                    controlMode = POWER;
                    break;
            }
        }

        switch (controlMode) {
            case POSITION:
                if (currentGamepad.dpad_up && !previousGamepad.dpad_up) {
                    positionStep++;
                }

                if (currentGamepad.dpad_down && !previousGamepad.dpad_down) {
                    positionStep--;
                }

                if (currentGamepad.dpad_right && !previousGamepad.dpad_left) {
                    positionPower += 0.05;
                }

                if (currentGamepad.dpad_left && !previousGamepad.dpad_left) {
                    positionPower -= 0.05;
                }

                break;
            case VELOCITY:
                if (currentGamepad.dpad_up && !previousGamepad.dpad_up) {
                    velocityStep++;
                }

                if (currentGamepad.dpad_down && !previousGamepad.dpad_down) {
                    velocityStep--;
                    if (velocityStep <= 1) velocityStep = 1;
                }
                break;
            default:
                break;
        }
    }

    private void controlMotor() {
        double leftStickY = gamepad1.left_stick_y * -1.0;

        switch (controlMode) {
            case POWER:
                testMotor.setMode(RUN_WITHOUT_ENCODER);
                testMotor.setPower(0.5);
                break;
            case VELOCITY:
                testMotor.setMode(RUN_USING_ENCODER);
                testMotor.setVelocity(leftStickY * velocityStep);
                break;
            case POSITION:
                int targetPosition = testMotor.getCurrentPosition();
                targetPosition += (int) leftStickY * positionStep;
                testMotor.setTargetPosition(targetPosition);
                testMotor.setMode(RUN_TO_POSITION);
                testMotor.setPower(positionPower);
                break;
        }
    }

    private void displayControls() {
        telemetry.addLine("Press X To Invert The Direction");
        telemetry.addLine("Press △ To Toggle The Zero Power Behaviour");
        telemetry.addLine("Press □ To Toggle The Control Mode");

        switch (controlMode) {
            case POWER:
                telemetry.addLine("Use The Left Joystick To Control The Power");
                break;
            case VELOCITY:
                telemetry.addLine("Use The Left Joystick To Control The Target Velocity");
                telemetry.addLine("Press Dpad Up To Increment The Velocity Step");
                telemetry.addLine("Press Dpad Down To Decrement The Velocity Step");
                break;
            case POSITION:
                telemetry.addLine("Use The Left Joystick To Control The Target Position");
                telemetry.addLine("Press Dpad Up/Down To Increment/Decrement The Position Step");
                telemetry.addLine("Press Dpad Left/Right To Increment/Decrement The Power");
                break;
        }
    }

    private void displayMotorTelemetry() {
        telemetry.addData("Direction", testMotor.getDirection());
        telemetry.addData("Current (AMPS)", testMotor.getCurrent(AMPS));
        telemetry.addData("Power", testMotor.getPower());
        telemetry.addData("Velocity (DEG/S)", testMotor.getVelocity(DEGREES));
        telemetry.addData("Run Mode", testMotor.getMode());
        telemetry.addData("Position", testMotor.getCurrentPosition());
        telemetry.addData("Target Position", testMotor.getTargetPosition());
        telemetry.addData("Zero Power Behavior", testMotor.getZeroPowerBehavior());
    }
}
