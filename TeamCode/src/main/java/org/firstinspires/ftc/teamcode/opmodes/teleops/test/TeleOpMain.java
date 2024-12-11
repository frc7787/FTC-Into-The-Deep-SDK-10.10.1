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

    private double INTAKE_OPEN_POSITION = 0.50;
    private double INTAKE_SUB_PRIMED_POSITION = 0.30;
    private double INTAKE_CLOSED_POSITION = 0.00;
    private double INTAKE_SPECIMEN_PICKUP_POSITION = 0.25;
    private double INTAKE_CLIPPING_POSITION = 0.22;

    private double BUCKET_VERTICAL_POSITION = 42.0;
    private double BUCKET_HORIZONTAL_POSITION = 5;
    private double BAR_VERTICAL_POSITION = 26.0;
    private double BAR_HORIZONTAL_POSITION = 2.0;

    private double SUB_VERTICAL_POSITION = 0.4;
    private double SUB_HORIZONTAL_POSITION = 2.0;

    private double NEUTRAL_VERTICAL_POSITION = 5.0;
    private double NEUTRAL_HORIZONTAL_POSITION = 3.0;

    private double GROUND_VERTICAL_POSITION = -2.0;
    private double GROUND_HORIZONTAL_POSITION = 0.0;

    private double HOVER_VERTICAL_POSITION = 0.3;

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
        }

        switch (armState) {
            case HOMING:
                if (arm.state() != Arm.ArmState.HOMING) { armState = ArmState.NEUTRAL; }
                break;
            case NEUTRAL:
            case BUCKET:
            case HOOKING:
            case GROUND:
                if (gamepad2.triangle) {
                    arm.setTargetPositionRobotCentric(BUCKET_HORIZONTAL_POSITION, BUCKET_VERTICAL_POSITION);
                    armState = ArmState.BUCKET;
                } else if (gamepad2.circle) {
                    arm.setTargetPositionRobotCentric(BAR_HORIZONTAL_POSITION, BAR_VERTICAL_POSITION);
                    armState = ArmState.HOOKING;
                } else if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                    arm.setTargetPositionRobotCentric(NEUTRAL_HORIZONTAL_POSITION, NEUTRAL_VERTICAL_POSITION);
                    arm.setIntakePosition(INTAKE_SUB_PRIMED_POSITION);
                    armState = ArmState.SUB;
                } else if (gamepad2.dpad_left) {
                    arm.setTargetPositionRobotCentric(GROUND_HORIZONTAL_POSITION, GROUND_VERTICAL_POSITION);
                    armState = ArmState.GROUND;
                }
                break;
            case SUB:
                if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                    arm.setTargetPositionRobotCentric(NEUTRAL_HORIZONTAL_POSITION, NEUTRAL_VERTICAL_POSITION);
                    arm.setIntakePosition(INTAKE_SUB_PRIMED_POSITION);
                    armState = ArmState.NEUTRAL;
                } else {
                    if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                       arm.setTargetPositionInches(arm.hExtensionTargetInches(), -1);
                       arm.setIntakePosition(INTAKE_CLOSED_POSITION);
                    } else if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
                        arm.setTargetPositionInches(arm.hExtensionTargetInches(), -3);
                        arm.setIntakePosition(INTAKE_SUB_PRIMED_POSITION);
                    } else {
                        double gamepadleftX = gamepad2.left_stick_x;
                        double gamepadleftY = -gamepad2.left_stick_y;

                        if (Math.abs(gamepadleftX) < 0.1) gamepadleftX = 0;
                        if (Math.abs(gamepadleftY) < 0.1) gamepadleftY = 0;

                        arm.manualControl(gamepadleftX, gamepadleftY, 8);
                    }
                }
                break;
        }

        telemetry.addData("Servo Position", arm.intakePosition());
        telemetry.addData("Arm State", armState);
        arm.debugPosition();
        arm.update();
    }

    private enum ArmState {
        HOVERING,
        INTAKING,
        BUCKET,
        HOOKING,
        GROUND,
        HOMING,
        NEUTRAL,
        SUB
    }
}
