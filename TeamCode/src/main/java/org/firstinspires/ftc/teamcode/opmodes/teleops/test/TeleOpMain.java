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

    private double BUCKET_VERTICAL_POSITION = 48.0;
    private double BUCKET_HORIZONTAL_POSITION = 18.0;
    private double BAR_VERTICAL_POSITION = 26.0;
    private double BAR_HORIZONTAL_POSITION = 13.0;

    private double SUB_VERTICAL_POSITION = 0.4;
    private double SUB_HORIZONTAL_POSITION = 2.0;

    private double NEUTRAL_VERTICAL_POSITION = 6.0;
    private double NEUTRAL_HORIZONTAL_POSITION = 8.0;

    private double GROUND_VERTICAL_POSITION = -2.0;
    private double GROUND_HORIZONTAL_POSITION = 0.0;

    private double HOVER_VERTICAL_POSITION = 0.3;

    private Gamepad previousGamepad, currentGamepad;

    private ArmState armState;

    @Override public void init() {
        drive = new MecanumDrive.Builder(hardwareMap)
                .setDriveMode(DriveMode.ROBOT_CENTRIC)
                .build();
        arm = new Arm(this);
        previousGamepad = new Gamepad();
        currentGamepad  = new Gamepad();

        armState = ArmState.HOMING;
    }

    @Override public void loop() {
        previousGamepad.copy(currentGamepad);
        currentGamepad.copy(gamepad2);

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
                if (arm.state() != Arm.ArmState.HOMING) {
                    arm.setTargetPositionInches(NEUTRAL_VERTICAL_POSITION, NEUTRAL_HORIZONTAL_POSITION);
                    armState = ArmState.NEUTRAL;
                }
                break;
            case NEUTRAL:
            case BUCKET:
            case HOOKING:
            case GROUND:
                if (gamepad2.triangle) {
                    arm.setTargetPositionInches(BUCKET_HORIZONTAL_POSITION, BUCKET_VERTICAL_POSITION);
                    armState = ArmState.BUCKET;
                } else if (gamepad2.circle) {
                    arm.setTargetPositionInches(BAR_HORIZONTAL_POSITION, BAR_VERTICAL_POSITION);
                    armState = ArmState.HOOKING;
                } else if (gamepad1.left_bumper) {
                    arm.setTargetPositionInches(NEUTRAL_HORIZONTAL_POSITION, NEUTRAL_VERTICAL_POSITION);
                    arm.setIntakePosition(INTAKE_SUB_PRIMED_POSITION);
                    armState = ArmState.SUB;
                } else if (gamepad2.dpad_left) {
                    arm.setTargetPositionInches(GROUND_HORIZONTAL_POSITION, GROUND_VERTICAL_POSITION);
                    armState = ArmState.GROUND;
                }
                break;
            case SUB:
                if (gamepad1.left_bumper) {
                    arm.setTargetPositionInches(NEUTRAL_HORIZONTAL_POSITION, NEUTRAL_VERTICAL_POSITION);
                    armState = ArmState.NEUTRAL;
                } else {
                    if (gamepad1.dpad_up) {
                        arm.setIntakePosition(INTAKE_OPEN_POSITION);
                    } else if (gamepad1.right_bumper) {
                        arm.setIntakePosition(INTAKE_SUB_PRIMED_POSITION);
                        arm.setTargetPositionInches(arm.hExtensionTargetInches(), GROUND_VERTICAL_POSITION);
                    } else {
                        arm.setIntakePosition(INTAKE_CLOSED_POSITION);
                        arm.setTargetPositionInches(arm.hExtensionTargetInches(), HOVER_VERTICAL_POSITION);
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
