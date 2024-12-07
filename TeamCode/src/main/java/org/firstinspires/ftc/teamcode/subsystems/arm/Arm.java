package org.firstinspires.ftc.teamcode.subsystems.arm;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.utility.*;

public class Arm {
    // ---------------------------------------------------------------------------------------------
    // Configuration
    // ---------------------------------------------------------------------------------------------

    private final double EXTENSION_TICKS_PER_INCH  = 49.07;
    private final double ROTATION_TICKS_PER_DEGREE = 17.95;

    // Physical Properties
    private final double INTAKE_LENGTH_INCHES = 0.0;
    private final double ROTATION_X_OFFSET_INCHES = 5;
    private final double ROTATION_Y_OFFSET_INCHES = 4.75;
    private final double ARM_STARTING_ANGLE_DEGREES = -11.0;

    private final double MIN_ROTATION_DEGREES = 0;
    private final double MAX_ROTATION_DEGREES = 90;
    private final double MIN_EXTENSION_INCHES = 0;
    private final double MAX_EXTENSION_INCHES = 45;

    private final double MIN_ROTATION_TICKS = MIN_ROTATION_DEGREES * ROTATION_TICKS_PER_DEGREE;
    private final double MAX_ROTATION_TICKS = MAX_ROTATION_DEGREES * ROTATION_TICKS_PER_DEGREE;

    private final double MIN_EXTENSION_TICKS = MIN_EXTENSION_INCHES * EXTENSION_TICKS_PER_INCH;
    private final double MAX_EXTENSION_TICKS = MAX_EXTENSION_INCHES * EXTENSION_TICKS_PER_INCH;

    // Software limitations
    private final double FORWARD_EXTENSION_LIMIT_INCHES = 60;
    private final double ROTATION_LIMIT_DEGREES         = 95;

    private final double MAX_EXTENSION_POWER = 1.0;
    private final double MIN_EXTENSION_POWER = -1.0;
    private final double MAX_ROTATION_POWER  = 1.0;
    private final double MIN_ROTATION_POWER  = -0.8;

    // Miscellaneous
    private final double EXTENSION_HOMING_POWER = -0.8;
    private final double ROTATION_HOMING_POWER  = -0.6;

    private final int EXTENSION_POSITION_THRESHOLD = 10;
    private final int ROTATION_POSITION_THRESHOLD  = 10;

    // Intake
    private final double INTAKE_OPEN_POSITION   = 0.0;
    private final double INTAKE_CLOSED_POSITION = 0.5;

    // ---------------------------------------------------------------------------------------------
    // Hardware
    // ---------------------------------------------------------------------------------------------

    private final DcMotorImplEx leaderExtensionMotor,
                                followerExtensionMotor,
                                rotationMotor;

    private final RevTouchSensor frontRotationLimitSwitch,
                                 backRotationLimitSwitch,
                                 extensionLimitSwitch;

    private final Servo intakeServo;

    // ---------------------------------------------------------------------------------------------
    // Global State
    // ---------------------------------------------------------------------------------------------

    private ArmState armState;
    private HomingState homingState;

    private int rotationTargetPosition, extensionTargetPosition;
    private int rotationPosition, extensionPosition;

    // ---------------------------------------------------------------------------------------------
    // Debug
    // ---------------------------------------------------------------------------------------------

    private final Telemetry telemetry;

    // ---------------------------------------------------------------------------------------------
    // Controllers
    // ---------------------------------------------------------------------------------------------

    private final PIDController extensionController
            = new PIDController(0.0087, 0, 0.00012);

    private final PIDController rotationController
            = new PIDController(0.0092, 0, 0.000012);

    // ---------------------------------------------------------------------------------------------
    // Construction
    // ---------------------------------------------------------------------------------------------

    public Arm(@NonNull OpMode opMode) {
        telemetry = opMode.telemetry;

        HardwareMap hardwareMap = opMode.hardwareMap;

        leaderExtensionMotor = hardwareMap.get(DcMotorImplEx.class, "leaderExtensionMotor");
        followerExtensionMotor = hardwareMap.get(DcMotorImplEx.class, "followerExtensionMotor");
        rotationMotor = hardwareMap.get(DcMotorImplEx.class, "rotationMotor");

        MotorUtility.setZeroPowerBehaviours(
                BRAKE, leaderExtensionMotor, followerExtensionMotor, rotationMotor);
        MotorUtility.reset(leaderExtensionMotor, followerExtensionMotor, rotationMotor);

        frontRotationLimitSwitch = hardwareMap.get(RevTouchSensor.class, "frontRotationLimitSwitch");
        backRotationLimitSwitch = hardwareMap.get(RevTouchSensor.class, "backRotationLimitSwitch");
        extensionLimitSwitch = hardwareMap.get(RevTouchSensor.class, "extensionLimitSwitch");

        intakeServo = hardwareMap.get(Servo.class, "intakeServo");
        intakeServo.setPosition(0.0);
        intakeServo.setDirection(Servo.Direction.FORWARD);

        extensionTargetPosition = 0;
        rotationTargetPosition  = 0;

        armState    = ArmState.HOMING;
        homingState = HomingState.START;
    }

    // ---------------------------------------------------------------------------------------------
    // Core
    // ---------------------------------------------------------------------------------------------

    public void update() {
        rotationPosition  = rotationMotor.getCurrentPosition();
        extensionPosition = leaderExtensionMotor.getCurrentPosition();

        switch (armState) {
            case HOMING:
                home();
                break;
            case NORMAL:
                if (extensionTargetPosition == 0 && extensionLimitSwitch.isPressed()) {
                    MotorUtility.reset(leaderExtensionMotor);
                }

                if (rotationTargetPosition == 0 && frontRotationLimitSwitch.isPressed()) {
                    MotorUtility.reset(rotationMotor);
                }

                double rotationPower
                        = rotationController.calculate(rotationPosition, rotationTargetPosition);
                rotationPower = Range.clip(rotationPower, MIN_ROTATION_POWER, MAX_ROTATION_POWER);
                double extensionPower
                        = extensionController.calculate(extensionPosition, extensionTargetPosition);
                extensionPower = Range.clip(extensionPower, MIN_EXTENSION_POWER, MAX_EXTENSION_POWER);

                if (rotationTargetPosition <= rotationPosition && !extensionAtPosition()) {
                    rotationPower = 0.0;
                } else if (!rotationAtPosition()) {
                    extensionPower = 0.0;
                }

                if (rotationAtPosition()) rotationPower = 0.0;
                if (extensionAtPosition()) extensionPower = 0.0;

                leaderExtensionMotor.setPower(extensionPower);
                followerExtensionMotor.setPower(extensionPower);
                rotationMotor.setPower(rotationPower);
                break;
        }
    }

    private void home() {
        switch (homingState) {
            case START:
                zeroIntake();
                if (extensionLimitSwitch.isPressed() && frontRotationLimitSwitch.isPressed()) {
                    MotorUtility.reset(leaderExtensionMotor, followerExtensionMotor, rotationMotor);
                    homingState = HomingState.COMPLETE;
                } else {
                    homingState = HomingState.INITIAL_RETRACTION;
                }
                break;
            case INITIAL_RETRACTION:
                if (extensionLimitSwitch.isPressed()) {
                    MotorUtility.reset(leaderExtensionMotor, followerExtensionMotor);
                    homingState = HomingState.SAFETY_EXTENSION;
                } else {
                    leaderExtensionMotor.setPower(EXTENSION_HOMING_POWER);
                    followerExtensionMotor.setPower(EXTENSION_HOMING_POWER);
                }
                break;
            case SAFETY_EXTENSION:
                int currentPosition = leaderExtensionMotor.getCurrentPosition();
                double power = extensionController.calculate(currentPosition, 300);
                leaderExtensionMotor.setPower(power);
                followerExtensionMotor.setPower(power);

                if (Math.abs(currentPosition - 300) <= 10) {
                    leaderExtensionMotor.setPower(0);
                    followerExtensionMotor.setPower(0);
                    homingState = HomingState.HOMING_ROTATION;
                }
            case HOMING_ROTATION:
                if (frontRotationLimitSwitch.isPressed()) {
                    MotorUtility.reset(rotationMotor);
                    homingState = HomingState.FINAL_RETRACTION;
                } else {
                    rotationMotor.setPower(ROTATION_HOMING_POWER);
                }
                break;
            case FINAL_RETRACTION:
                if (extensionLimitSwitch.isPressed()) {
                    MotorUtility.reset(rotationMotor);
                    homingState = HomingState.COMPLETE;
                } else {
                    leaderExtensionMotor.setPower(EXTENSION_HOMING_POWER);
                    followerExtensionMotor.setPower(EXTENSION_HOMING_POWER);
                }
                break;
            case COMPLETE:
                armState = ArmState.NORMAL;
                rotationTargetPosition  = 0;
                extensionTargetPosition = 0;
                MotorUtility.reset(rotationMotor, leaderExtensionMotor, followerExtensionMotor);
                break;
        }
    }

    public void setTargetPosition(double hExtensionInches, double vExtensionInches) {
        double extensionInches = Math.sqrt(
                Math.pow(hExtensionInches, 2.0) +
                Math.pow(vExtensionInches, 2.0) +
                - Math.pow(1.5, 2)
        ) - 11.0;

        double rotationDegrees = Math.toDegrees(
                Math.atan(hExtensionInches / vExtensionInches) - Math.atan(1.5 / (extensionInches + 11)));

        rotationTargetPosition = (int) ((rotationDegrees + 11) * ROTATION_TICKS_PER_DEGREE) + 114;
        extensionTargetPosition = (int) ((extensionInches+2 )* EXTENSION_TICKS_PER_INCH);
    }

    public void setExtensionTargetPosition(double targetInches) {
        targetInches = Range.clip(targetInches, MIN_EXTENSION_INCHES, MAX_EXTENSION_INCHES);
        this.extensionTargetPosition = (int) (targetInches * EXTENSION_TICKS_PER_INCH);
    }

    public void setRotationTargetPosition(double targetDegrees) {
        targetDegrees = Range.clip(targetDegrees, MIN_ROTATION_DEGREES, MAX_ROTATION_DEGREES);
        this.rotationTargetPosition = (int) (targetDegrees * ROTATION_TICKS_PER_DEGREE);
    }

    public void setIntakePosition(double position) {
        intakeServo.setPosition(position);
    }

    public void zeroIntake() { intakeServo.setPosition(0.0); }

    // ---------------------------------------------------------------------------------------------
    // Getters
    // ---------------------------------------------------------------------------------------------

    public ArmState state() { return armState; }

    public int rotationPosition() { return rotationMotor.getCurrentPosition(); }

    public int extensionPosition() { return leaderExtensionMotor.getCurrentPosition(); }

    public double degrees() { return rotationMotor.getCurrentPosition() / ROTATION_TICKS_PER_DEGREE ; }

    public double inches() {
        return (leaderExtensionMotor.getCurrentPosition() / EXTENSION_TICKS_PER_INCH)
                + INTAKE_LENGTH_INCHES;
    }

    public int rotationTargetPosition() { return rotationTargetPosition; }

    public int extensionTargetPosition() { return extensionTargetPosition; }

    public double targetAngleDegrees() {
        return rotationTargetPosition / ROTATION_TICKS_PER_DEGREE;
    }

    public double targetExtensionInches() {
        return extensionTargetPosition / EXTENSION_TICKS_PER_INCH;
    }

    public double currentAmps() {
        return leaderExtensionMotor.getCurrent(CurrentUnit.AMPS)
               + followerExtensionMotor.getCurrent(CurrentUnit.AMPS)
               + rotationMotor.getCurrent(CurrentUnit.AMPS);
    }

    public boolean rotationAtPosition() {
        return Math.abs(rotationPosition - rotationTargetPosition) <= ROTATION_POSITION_THRESHOLD;
    }

    public boolean extensionAtPosition() {
        return Math.abs(extensionPosition - extensionTargetPosition) <= EXTENSION_POSITION_THRESHOLD;
    }

    public boolean isAtPosition() {
        return Math.abs(rotationPosition - rotationTargetPosition) <= ROTATION_POSITION_THRESHOLD
                && Math.abs(extensionPosition - extensionTargetPosition) <= EXTENSION_POSITION_THRESHOLD;
    }

    // ---------------------------------------------------------------------------------------------
    // Debug
    // ---------------------------------------------------------------------------------------------

    public void debugGlobal() {
        telemetry.addLine("----- Debug Global -----");
        telemetry.addData("Rotation Limit Switch Pressed", frontRotationLimitSwitch.isPressed());
        telemetry.addData("Extension Limit Switch Pressed", extensionLimitSwitch.isPressed());
        telemetry.addData("Arm State", armState);
        telemetry.addData("Homing State", homingState);
    }

    public void debugSetPowers(double rotationPower, double extensionPower) {
        rotationMotor.setPower(rotationPower);
        leaderExtensionMotor.setPower(extensionPower);
        followerExtensionMotor.setPower(extensionPower);
    }

    public void debugPosition() {
        telemetry.addLine("----- Extension -----");
        telemetry.addData("Position", leaderExtensionMotor.getCurrentPosition());
        telemetry.addData("Target Position", extensionTargetPosition);
        telemetry.addData("Inches", leaderExtensionMotor.getCurrentPosition() / EXTENSION_TICKS_PER_INCH);
        telemetry.addData("Target Inches", targetExtensionInches());
        telemetry.addData("Power", leaderExtensionMotor.getPower());
        telemetry.addData("At Position", extensionAtPosition());
        telemetry.addLine("----- Rotation -----");
        telemetry.addData("Position", rotationMotor.getCurrentPosition());
        telemetry.addData("Target Position", rotationTargetPosition);
        telemetry.addData("Degrees", rotationMotor.getCurrentPosition() / ROTATION_TICKS_PER_DEGREE);
        telemetry.addData("Target Degrees", targetAngleDegrees());
        telemetry.addData("Power", rotationMotor.getPower());
        telemetry.addData("At Position", rotationAtPosition());
    }

    public void debugCurrent() {
        double extensionCurrent = leaderExtensionMotor.getCurrent(CurrentUnit.AMPS)
                                + followerExtensionMotor.getCurrent(CurrentUnit.AMPS);
        double rotationCurrent  = rotationMotor.getCurrent(CurrentUnit.AMPS);

        telemetry.addLine("----- Current (AMPS) -----");
        telemetry.addData("Extension", extensionCurrent);
        telemetry.addData("Rotation", rotationCurrent);
        telemetry.addData("Total", extensionCurrent + rotationCurrent);
    }

    // ---------------------------------------------------------------------------------------------
    // State Enums
    // ---------------------------------------------------------------------------------------------

    public enum ArmState {
        HOMING,
        NORMAL,
    }

    public enum HomingState {
        START,
        INITIAL_RETRACTION,
        SAFETY_EXTENSION,
        HOMING_ROTATION,
        FINAL_RETRACTION,
        COMPLETE
    }
}
