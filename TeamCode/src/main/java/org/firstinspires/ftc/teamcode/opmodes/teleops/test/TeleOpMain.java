package org.firstinspires.ftc.teamcode.opmodes.teleops.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

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

    private final double BUCKET_VERTICAL_POSITION = 39.0;
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

    private final double WALL_VERTICAL_INCHES = 10;
    private final double WALL_HORIZONTAL_INCHES = 2;

    private Gamepad previousGamepad2, currentGamepad2, previousGamepad1, currentGamepad1;

    private ArmState armState;

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
                if (gamepad2.triangle) {
                    arm.setTargetPositionInchesRobotCentric(BUCKET_HORIZONTAL_POSITION, BUCKET_VERTICAL_POSITION);
                } else if (gamepad2.square) {
                    arm.setTargetPositionInchesRobotCentric(BAR_HORIZONTAL_POSITION, BAR_VERTICAL_POSITION);
                } else if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                    arm.setTargetPositionInchesRobotCentric(NEUTRAL_HORIZONTAL_POSITION, NEUTRAL_VERTICAL_POSITION);
                    arm.setIntakePosition(INTAKE_SUB_PRIMED_POSITION);
                    armState = ArmState.SUB;
                } else if (gamepad2.dpad_left) {
                    arm.setTargetPositionInchesRobotCentric(WALL_HORIZONTAL_INCHES, WALL_VERTICAL_INCHES);
                } else if (currentGamepad2.dpad_down) {
                    arm.setExtensionTargetPosition(arm.targetInches() - 6);
                } else {
                    double gamepadleftX = gamepad2.left_stick_x;
                    double gamepadleftY = -gamepad2.left_stick_y;

                    if (Math.abs(gamepadleftX) < 0.1) gamepadleftX = 0;
                    if (Math.abs(gamepadleftY) < 0.1) gamepadleftY = 0;

                    arm.manualControl(gamepadleftX, gamepadleftY, 10);
                }
                break;
            case SUB:
                arm.setIntakePosition(INTAKE_SUB_PRIMED_POSITION);

                if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                    arm.setTargetPositionInchesRobotCentric(NEUTRAL_HORIZONTAL_POSITION, NEUTRAL_VERTICAL_POSITION);
                    armState = ArmState.NEUTRAL;
                    arm.setIntakePosition(INTAKE_CLOSED_POSITION);
                } else {
                    if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                       arm.setTargetPositionInches(arm.hExtensionTargetInches(), -1);
                    } else if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
                        arm.setTargetPositionInches(arm.hExtensionTargetInches(), -5);
                    } else {
                        double gamepadleftX = gamepad2.left_stick_x;
                        double gamepadleftY = -gamepad2.left_stick_y;

                        if (Math.abs(gamepadleftX) < 0.1) gamepadleftX = 0;
                        if (Math.abs(gamepadleftY) < 0.1) gamepadleftY = 0;

                        arm.manualControl(gamepadleftX, gamepadleftY, 10);
                    }
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
