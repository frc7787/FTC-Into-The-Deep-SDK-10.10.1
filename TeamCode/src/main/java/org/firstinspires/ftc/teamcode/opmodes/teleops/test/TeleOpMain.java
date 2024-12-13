package org.firstinspires.ftc.teamcode.opmodes.teleops.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.LED;

import org.firstinspires.ftc.teamcode.roadrunner.DriveMode;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;

@TeleOp(group="$")
public class TeleOpMain extends OpMode {
    private MecanumDrive drive;
    private Arm arm;

    private final double INTAKE_OPEN_POSITION = 0.50;
    private final double INTAKE_SUB_PRIMED_POSITION = 0.35;
    private final double INTAKE_CLOSED_POSITION = 0.00;
    private final double INTAKE_SPECIMEN_PICKUP_POSITION = 0.25;
    private final double INTAKE_CLIPPING_POSITION = 0.22;

    private final double BUCKET_VERTICAL_POSITION = 40.0;
    private final double BUCKET_HORIZONTAL_POSITION = 5;
    private final double BAR_VERTICAL_POSITION = 24.5;
    private final double BAR_HORIZONTAL_POSITION = 1.5;

    private final double SUB_VERTICAL_POSITION = 0.4;
    private final double SUB_HORIZONTAL_POSITION = 2.0;

    private final double NEUTRAL_VERTICAL_POSITION = 6.0;
    private final double NEUTRAL_HORIZONTAL_POSITION = 3.0;

    private final double GROUND_VERTICAL_POSITION = -2.0;
    private final double GROUND_HORIZONTAL_POSITION = 0.0;

    private final double HOVER_VERTICAL_POSITION = 0.3;

    private final double WALL_VERTICAL_INCHES = 11;
    private final double WALL_HORIZONTAL_INCHES = 2;

    private Gamepad previousGamepad2, currentGamepad2, previousGamepad1, currentGamepad1;

    private ArmState armState;

    private LED leftLEDChannelOne, leftLEDChannelTwo, rightLEDChannelOne, rightLEDChannelTwo;

    @Override public void init() {
        drive = new MecanumDrive.Builder(hardwareMap)
                .setDriveMode(DriveMode.ROBOT_CENTRIC)
                .build();
        arm = new Arm(this);
        previousGamepad2 = new Gamepad();
        currentGamepad2 = new Gamepad();
        previousGamepad1 = new Gamepad();
        currentGamepad1 = new Gamepad();

        armState = ArmState.HOMING;

        leftLEDChannelOne = hardwareMap.get(LED.class, "leftLEDChannelOne");
        leftLEDChannelTwo = hardwareMap.get(LED.class, "leftLEDChannelTwo");
        rightLEDChannelOne = hardwareMap.get(LED.class, "rightLEDChannelOne");
        rightLEDChannelTwo = hardwareMap.get(LED.class, "rightLEDChannelTwo");
    }

    @Override public void loop() {
        previousGamepad2.copy(currentGamepad2);
        currentGamepad2.copy(gamepad2);
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        double leftStickY = gamepad1.left_stick_y * -1.0;
        double leftStickX = gamepad1.left_stick_x;
        double rightStickX = gamepad1.right_stick_x;

        drive.drive(
                leftStickY * Math.abs(leftStickY),
                leftStickX * Math.abs(leftStickX),
                rightStickX * Math.abs(rightStickX)
        );

        if (gamepad2.left_bumper) {
            arm.setIntakePosition(INTAKE_OPEN_POSITION);
        } else if (gamepad2.right_bumper) {
            arm.setIntakePosition(INTAKE_CLOSED_POSITION);
        }

        switch (armState) {
            case HOMING:
                if (arm.state() != Arm.ArmState.HOMING) { armState = ArmState.NEUTRAL; }
                break;
            case NEUTRAL:
                leftLEDChannelOne.enable(false);
                leftLEDChannelTwo.enable(false);
                rightLEDChannelOne.enable(false);
                rightLEDChannelTwo.enable(false);
                gamepad1.stopRumble();
                if (gamepad2.triangle) {
                    arm.setTargetPositionInchesRobotCentric(BUCKET_HORIZONTAL_POSITION, BUCKET_VERTICAL_POSITION);
                } else if (gamepad2.square) {
                    arm.setTargetPositionInchesRobotCentric(BAR_HORIZONTAL_POSITION, BAR_VERTICAL_POSITION);
                } else if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                    arm.setTargetPositionInchesRobotCentric(NEUTRAL_HORIZONTAL_POSITION, NEUTRAL_VERTICAL_POSITION);
                    arm.setIntakePosition(INTAKE_SUB_PRIMED_POSITION);
                    armState = ArmState.SUB;
                } else if (gamepad2.dpad_left) {
                    arm.setTargetPositionInchesRobotCentric(GROUND_HORIZONTAL_POSITION, GROUND_VERTICAL_POSITION);
                } else if (gamepad2.circle) {
                    arm.setTargetPositionInches(9.8, 6);
                } else if (gamepad2.cross) {
                    arm.setTargetPositionInchesRobotCentric(-3, 20);
                } else if (gamepad2.dpad_up) {
                    arm.setTargetPositionInches(9.8, 8.5);
                } else if (gamepad2.dpad_down) {
                    arm.setTargetPositionInchesRobotCentric(1.5, 18);
                } else {
                    double gamepadleftX = gamepad2.left_stick_x;
                    double gamepadleftY = -gamepad2.left_stick_y;

                    if (Math.abs(gamepadleftX) < 0.1) gamepadleftX = 0;
                    if (Math.abs(gamepadleftY) < 0.2) gamepadleftY = 0;

                    arm.manualControl(gamepadleftX, gamepadleftY, 10);
                }
                break;
            case SUB:
                leftLEDChannelOne.enable(true);
                leftLEDChannelTwo.enable(true);
                rightLEDChannelOne.enable(true);
                rightLEDChannelTwo.enable(true);
                gamepad2.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);
                arm.setIntakePosition(INTAKE_SUB_PRIMED_POSITION);

                if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                    arm.setTargetPositionInchesRobotCentric(NEUTRAL_HORIZONTAL_POSITION, NEUTRAL_VERTICAL_POSITION);
                    armState = ArmState.NEUTRAL;
                    arm.setIntakePosition(INTAKE_CLOSED_POSITION);
                } else {
                    if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                       arm.setTargetPositionInches(arm.hExtensionTargetInches(), 1);
                    } else if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
                        arm.setTargetPositionInches(arm.hExtensionTargetInches(), -5);
                    }
                    double gamepadleftX = gamepad1.right_trigger - gamepad1.left_trigger;
                    double gamepadleftY = -gamepad1.right_stick_y;

                    if (Math.abs(gamepadleftX) < 0.1) gamepadleftX = 0;
                    if (Math.abs(gamepadleftY) < 0.2) gamepadleftY = 0;

                    arm.manualControl(gamepadleftY, gamepadleftX, 10);
                }
                break;
        }



        telemetry.addData("Servo Position", arm.intakePosition());
        telemetry.addData("Arm State", armState);
        arm.debugPosition();
        arm.debugGlobal();
        arm.update();
    }

    private enum ArmState {
        INTAKING,
        HOMING,
        NEUTRAL,
        SUB
    }
}
