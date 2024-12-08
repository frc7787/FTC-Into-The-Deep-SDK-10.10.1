package org.firstinspires.ftc.teamcode.opmodes.teleops.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.roadrunner.DriveMode;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;

@TeleOp
public class TeleOpMain extends OpMode {
    private MecanumDrive drive;
    private Arm arm;

    private Gamepad previousGamepad2, currentGamepad2;

    private double LOW_BUCKET_EXTENSION_INCHES = 25;
    private double LOW_BUCKET_ROTATION_DEGREES = 80.5;

    private double HIGH_BUCKET_EXTENSION_INCHES = 42.5;
    private double HIGH_BUCKET_ROTATION_DEGREES = 90;

    private double LOW_BAR_EXTENSION_INCHES = 8;
    private double LOW_BAR_ROTATION_DEGREES = 63;

    private double HIGH_BAR_EXTENSION_INCHES = 0;
    private double HIGH_BAR_ROTATION_DEGREES = 0;

    private HookingState hookingState;

    @Override public void init() {
        drive = new MecanumDrive.Builder(hardwareMap)
                .setDriveMode(DriveMode.ROBOT_CENTRIC)
                .build();
        arm = new Arm(this);
        previousGamepad2 = new Gamepad();
        currentGamepad2 = new Gamepad();
        hookingState = HookingState.START;
    }

    @Override public void loop() {
        arm.zeroIntake();
        previousGamepad2.copy(currentGamepad2);
        currentGamepad2.copy(gamepad2);

        if (gamepad2.dpad_down) {
            arm.setTargetPosition(0,0);
        } else if (gamepad2.triangle) {
            arm.setTargetPosition(HIGH_BUCKET_EXTENSION_INCHES, HIGH_BUCKET_ROTATION_DEGREES);
        } else if (gamepad2.square) {
            arm.setTargetPosition(LOW_BUCKET_EXTENSION_INCHES, LOW_BUCKET_ROTATION_DEGREES);
        } else if (gamepad2.cross){
            arm.setTargetPosition(LOW_BAR_EXTENSION_INCHES, LOW_BAR_ROTATION_DEGREES);
        } else if (gamepad2.circle) {
            arm.setTargetPosition(HIGH_BAR_EXTENSION_INCHES, HIGH_BAR_ROTATION_DEGREES);
        } else if (gamepad2.dpad_left) {
            telemetry.addData("Rotation", Math.abs(LOW_BAR_ROTATION_DEGREES - arm.degrees()));
            if (Math.abs(LOW_BAR_ROTATION_DEGREES - arm.degrees()) <= 2.0) {
                telemetry.addLine("High Bar!");
                hook(BarHeight.LOW, 0, 0);
            } else if (Math.abs(HIGH_BAR_ROTATION_DEGREES - arm.degrees()) < 2.0) {
                hook(BarHeight.HIGH, 0, 0);
                telemetry.addLine("Low Bar!");
            }
            telemetry.addLine("No Bar :(");
        }

        arm.debugPosition();
        arm.update();
    }

    private void hook(BarHeight barHeight, double rotationFinalTarget, double extensionFinalTarget) {
        switch (hookingState) {
            case START:
                arm.setRotationTargetPosition(barHeight.angleToHook());
                hookingState = HookingState.RETRACTING;
                break;
            case ROTATING:
                if (arm.rotationAtPosition()) {
                    arm.setExtensionTargetPosition(arm.inches() - 5.0);
                    hookingState = HookingState.RETRACTING;
                }
                break;
            case RETRACTING:
                if (arm.extensionAtPosition()) { hookingState = HookingState.COMPLETE; }
                break;
            case COMPLETE:
                arm.setTargetPosition(rotationFinalTarget, extensionFinalTarget);
                break;
        }
        telemetry.addData("Hooking State", hookingState);
    }

    private enum HookingState {
        START,
        ROTATING,
        RETRACTING,
        COMPLETE
    }

    private enum BarHeight {
        LOW,
        HIGH;

        public double angleToHook() {
            switch (this) {
                case LOW:
                    return 48.0;
                case HIGH:
                    return 0.0;
            }
            throw new RuntimeException("Enum must be one of its ordinals???");
        }
    }
}
