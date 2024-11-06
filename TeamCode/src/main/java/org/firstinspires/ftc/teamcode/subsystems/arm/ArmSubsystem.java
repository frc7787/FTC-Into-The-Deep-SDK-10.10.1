package org.firstinspires.ftc.teamcode.subsystems.arm;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.*;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utility.MotorUtility;
import org.firstinspires.ftc.teamcode.utility.PIDController;

import static org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS;
import static org.firstinspires.ftc.teamcode.Constants.ArmConstants.*;

public final class ArmSubsystem extends SubsystemBase {
    private enum HomingState {
        START,
        HOMING_ROTATION,
        HOMING_EXTENSION,
        FINISHED
    }

    private enum ArmState {
        HOMING,
        AT_POS,
        TO_POS
    }

    private double maxRotationCurrent = 0;
    private double maxExtensionCurrent = 0;

    private final RevTouchSensor extensionLimitSwitch, rotationLimitSwitch;
    private final DcMotorImplEx rotationMotor, extensionMotorOne, extensionMotorTwo;
    public int extensionTargetPosition, rotationTargetPosition;

    private final PIDController rotationController  = new PIDController(0.004, 0.0, 0.00004);
    private final PIDController extensionController = new PIDController(0.006, 0.0, 0.00006);

    private HomingState homingState;
    private ArmState armState;

    private final Telemetry telemetry;

    public ArmSubsystem(@NonNull OpMode opMode) {
        telemetry = opMode.telemetry;
        HardwareMap hardwareMap = opMode.hardwareMap;

        this.extensionMotorOne    = hardwareMap.get(DcMotorImplEx.class, "extensionMotorOne");
        this.extensionMotorTwo    = hardwareMap.get(DcMotorImplEx.class, "extensionMotorTwo");
        this.rotationMotor        = hardwareMap.get(DcMotorImplEx.class, "rotationMotor");
        this.extensionLimitSwitch = hardwareMap.get(RevTouchSensor.class, "extensionLimitSwitch");
        this.rotationLimitSwitch  = hardwareMap.get(RevTouchSensor.class, "rotationLimitSwitch");

        MotorUtility.reset(extensionMotorOne, extensionMotorTwo, rotationMotor);
        MotorUtility.setModes(
                RUN_WITHOUT_ENCODER, extensionMotorOne, extensionMotorTwo, rotationMotor);

        extensionTargetPosition = 0;
        rotationTargetPosition  = 0;

        homingState = HomingState.START;
        armState    = ArmState.AT_POS;
    }

    @Override public void periodic() {
        switch (armState) {
            case HOMING:
                home();
                break;
            case AT_POS:
                // Nothing here but chickens
                break;
            case TO_POS:
                double extensionAngleRadians = rotationTicksToRadians(rotationTargetPosition);
                double extensionDistanceTicks
                        = extensionTargetPosition * Math.cosh(extensionAngleRadians);

                if (extensionDistanceTicks > EXTENSION_LIMIT) {
                    extensionTargetPosition
                            = (int) (extensionTargetPosition * Math.cosh(extensionAngleRadians));
                }

                rotate(rotationTargetPosition);
                extend(extensionTargetPosition);
        }
    }

    public void setArmPower(double extensionPower, double rotationPower) {
        extensionMotorOne.setPower(extensionPower);
        extensionMotorTwo.setPower(extensionPower);
        rotationMotor.setPower(rotationPower);
    }

    private void extend(int targetPosition, double maxPower) {
        int currentPosition = extensionMotorOne.getCurrentPosition();
        double power = extensionController.calculate(currentPosition, targetPosition);
        power = Range.clip(power, -maxPower, maxPower);
        extensionMotorOne.setPower(power);
        extensionMotorTwo.setPower(power);
    }

    private void extend(int targetPosition) {
        extend(targetPosition, 1.0);
    }

    private void rotate(int targetPosition, double maxPower) {
        int currentPosition = rotationMotor.getCurrentPosition();
        double power = rotationController.calculate(currentPosition, targetPosition);
        power = Range.clip(power, -maxPower, maxPower);
        rotationMotor.setPower(power);
    }

    private void rotate(int targetPosition) {
        rotate(targetPosition, 1.0);
    }

    private void home() {
        switch (homingState) {
            case START:
                if (extensionLimitSwitch.isPressed() && rotationLimitSwitch.isPressed()) {
                    MotorUtility.reset(extensionMotorOne, extensionMotorTwo, rotationMotor);
                    homingState = HomingState.FINISHED;
                } else if (extensionLimitSwitch.isPressed()) {
                    MotorUtility.reset(extensionMotorOne, extensionMotorTwo);
                    homingState = HomingState.HOMING_ROTATION;
                } else if (rotationLimitSwitch.isPressed()) {
                    MotorUtility.reset(rotationMotor);
                    homingState = HomingState.HOMING_EXTENSION;
                } else {
                    homingState = HomingState.HOMING_ROTATION;
                }
                break;
            case HOMING_ROTATION:
                if (rotationLimitSwitch.isPressed()) {
                    rotationMotor.setPower(0.0);
                    MotorUtility.reset(rotationMotor);
                    homingState = HomingState.HOMING_EXTENSION;
                } else {
                    int currentPosition = rotationMotor.getCurrentPosition();

                    if (currentPosition >= ROTATION_HOME_POSITION) {
                        rotationMotor.setPower(-0.4);
                    } else {
                        rotationMotor.setPower(0.4);
                    }
                }
                break;
            case FINISHED:
                extensionMotorOne.setMode(RUN_WITHOUT_ENCODER);
                extensionMotorTwo.setMode(RUN_WITHOUT_ENCODER);
                rotationMotor.setMode(RUN_WITHOUT_ENCODER);
                armState = ArmState.AT_POS;
                break;
        }
    }

    public int extensionCurrentPosition() {
        return extensionMotorOne.getCurrentPosition();
    }

    public void debugAll() {
       debugGlobal();
       debugRotation();
       debugExtension();
    }

    public void debugGlobal() {
        telemetry.addLine("----- Global -----");
        telemetry.addData("Arm State", armState);
        telemetry.addData("Homing State", homingState);
        telemetry.addData("Rotation Limit Switch Pressed", rotationLimitSwitch.isPressed());
        telemetry.addData("Extension Limit Switch Pressed", extensionLimitSwitch.isPressed());
    }

    public void debugRotation() {
        telemetry.addLine("----- Rotation Motor One -----");
        telemetry.addData("Power", rotationMotor.getPower());
        telemetry.addData("Position", rotationMotor.getCurrentPosition());
        telemetry.addData("Target Position (Subsystem)", rotationTargetPosition);
        telemetry.addData("Target Position (Motor)", rotationMotor.getTargetPosition());
        telemetry.addData("Current (AMPS)", rotationMotor.getCurrent(AMPS));
        telemetry.addData("Direction", rotationMotor.getDirection());
    }

    public void debugExtensionMotorOne() {
        telemetry.addLine("----- Extension Motor One -----");
        telemetry.addData("Power", extensionMotorOne.getPower());
        telemetry.addData("Position", extensionMotorOne.getCurrentPosition());
        telemetry.addData("Target Position (Subsystem)", extensionTargetPosition);
        telemetry.addData("Target Position (Motor)", extensionMotorOne.getTargetPosition());
        telemetry.addData("Current (AMPS)", extensionMotorOne.getCurrent(AMPS));
        telemetry.addData("Direction", extensionMotorOne.getDirection());
    }

    public void debugExtensionMotorTwo() {
        telemetry.addLine("----- Extension Motor Two -----");
        telemetry.addData("Power", extensionMotorTwo.getPower());
        telemetry.addData("Position", extensionMotorTwo.getCurrentPosition());
        telemetry.addData("Target Position (Subsystem)", extensionTargetPosition);
        telemetry.addData("Target Position", extensionMotorTwo.getTargetPosition());
        telemetry.addData("Current (AMPS)", extensionMotorTwo.getCurrent(AMPS));
        telemetry.addData("Direction", extensionMotorTwo.getDirection());
    }

    public void debugExtension() {
        debugExtensionMotorOne();
        debugExtensionMotorTwo();
    }

    public void displayCurrent() {
       double extensionCurrentAverage
               = extensionMotorOne.getCurrent(AMPS) + extensionMotorTwo.getCurrent(AMPS);
       double rotationCurrentAverage
               = rotationMotor.getCurrent(AMPS);

       if (extensionCurrentAverage > maxExtensionCurrent) {
           maxExtensionCurrent = extensionCurrentAverage;
       }

       if (rotationCurrentAverage > maxRotationCurrent) {
           maxRotationCurrent = rotationCurrentAverage;
       }

       telemetry.addData("Max Rotation Current", maxRotationCurrent);
       telemetry.addData("Max Extension Current", maxExtensionCurrent);
       telemetry.addData("Rotation Current", rotationCurrentAverage);
       telemetry.addData("Extension Current", extensionCurrentAverage);
    }

    private double rotationTicksToRadians(int ticks) {
       return ticks;
    }
}
