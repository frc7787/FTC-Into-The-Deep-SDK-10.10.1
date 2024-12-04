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

// TODO -- Manual Control
// TODO -- Automatic Hook Sequence
// TODO -- Extension Limit

public class Arm {
    // ---------------------------------------------------------------------------------------------
    // Configuration
    // ---------------------------------------------------------------------------------------------

    private final double EXTENSION_TICKS_PER_INCH  = 64.56;
    private final double ROTATION_TICKS_PER_DEGREE = 17.95;

    // Physical Properties
    private final double INTAKE_LENGTH_INCHES = 0.0;
    private final double ROTATION_X_OFFSET_INCHES = 5;
    private final double ROTATION_Y_OFFSET_INCHES = 4.75;
    private final double ARM_GUIDE_OFFSET_DEGREES = -5.0; // TODO name this better

    private final double MIN_ROTATION_DEGREES = 0;
    private final double MAX_ROTATION_DEGREES = 120;
    private final double MIN_EXTENSION_INCHES = 0;
    private final double MAX_EXTENSION_INCHES = 60;

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
    private final double EXTENSION_HOMING_POWER = -1.0;
    private final double ROTATION_HOMING_POWER  = -1.0;

    private final int EXTENSION_POSITION_THRESHOLD = 10;
    private final int ROTATION_POSITION_THRESHOLD  = 10;

    // Intake
    private final double INTAKE_OPEN_POSITION   = 0.0;
    private final double INTAKE_CLOSED_POSITION = 0.3;

    // ---------------------------------------------------------------------------------------------
    // Hardware
    // ---------------------------------------------------------------------------------------------

    private final DcMotorImplEx leaderExtensionMotor,
            followerExtensionMotor,
                                rotationMotor;

    private final RevTouchSensor rotationLimitSwitch,
                                 extensionLimitSwitch;

    private final Servo intakeServo;

    // ---------------------------------------------------------------------------------------------
    // Global State
    // ---------------------------------------------------------------------------------------------

    private ArmState armState;
    private HomingState homingState;

    private int rotationTargetPosition;
    private int extensionTargetPosition;

    private int rotationPosition;
    private int extensionPosition;

    private double rotationDegrees;
    private double extensionInches;
    private double extensionDisplacementInches;

    // ---------------------------------------------------------------------------------------------
    // Debug
    // ---------------------------------------------------------------------------------------------

    private final Telemetry telemetry;

    // ---------------------------------------------------------------------------------------------
    // Controllers
    // ---------------------------------------------------------------------------------------------

    private final PIDController extensionController
            = new PIDController(0.007, 0, 0.00009);

    private final PIDController rotationController
            = new PIDController(0.008, 0, 0);

    // ---------------------------------------------------------------------------------------------
    // Construction
    // ---------------------------------------------------------------------------------------------

    public Arm(@NonNull OpMode opMode) {
        telemetry = opMode.telemetry;

        HardwareMap hardwareMap = opMode.hardwareMap;

        leaderExtensionMotor
                = hardwareMap.get(DcMotorImplEx.class, "leaderExtensionMotor");
        followerExtensionMotor
                = hardwareMap.get(DcMotorImplEx.class, "followerExtensionMotor");
        rotationMotor
                = hardwareMap.get(DcMotorImplEx.class, "rotationMotor");

        MotorUtility.setZeroPowerBehaviours(
                BRAKE, leaderExtensionMotor, followerExtensionMotor, rotationMotor);
        MotorUtility.reset(leaderExtensionMotor, followerExtensionMotor, rotationMotor);

        rotationLimitSwitch = hardwareMap.get(RevTouchSensor.class, "rotationLimitSwitch");
        extensionLimitSwitch = hardwareMap.get(RevTouchSensor.class, "extensionLimitSwitch");

        intakeServo = hardwareMap.get(Servo.class, "intakeServo");
        intakeServo.setPosition(0.0);

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

        rotationDegrees = rotationPosition / ROTATION_TICKS_PER_DEGREE;
        extensionInches = (extensionPosition / EXTENSION_TICKS_PER_INCH)
                        + INTAKE_LENGTH_INCHES;

        if (this.extensionTargetPosition == 0 && extensionLimitSwitch.isPressed()) {
            MotorUtility.reset(leaderExtensionMotor);
        }

        if (this.rotationTargetPosition == 0 && rotationLimitSwitch.isPressed()) {
            MotorUtility.reset(rotationMotor);
        }

        switch (armState) {
            case HOMING:
                home();
                break;
            case AT_POS: // TODO Consider using PID to hold position
                if (!isAtPosition()) {
                    armState = ArmState.TO_POS;
                } else {
                    MotorUtility.setPowers(0, rotationMotor, leaderExtensionMotor, followerExtensionMotor);
                }
                break;
            case TO_POS:
                double rotationPower = rotationController.calculate(rotationPosition, rotationTargetPosition);
                rotationPower = Range.clip(rotationPower, MIN_ROTATION_POWER , MAX_ROTATION_POWER);
                rotationMotor.setPower(rotationPower);

                if (rotationAtPosition()) {
                    double extensionPower
                            = extensionController.calculate(extensionPosition, extensionTargetPosition);
                    extensionPower = Range.clip(extensionPower, MIN_EXTENSION_POWER, MAX_EXTENSION_POWER);
                    leaderExtensionMotor.setPower(extensionPower);
                    followerExtensionMotor.setPower(extensionPower);
                }

                if (isAtPosition()) armState = ArmState.AT_POS;
                break;
        }
    }

    private void home() {
        switch (homingState) {
            case START:
                if (extensionLimitSwitch.isPressed() && rotationLimitSwitch.isPressed()) {
                    MotorUtility.reset(leaderExtensionMotor, followerExtensionMotor, rotationMotor);
                    homingState = HomingState.COMPLETE;
                } else {
                    homingState = HomingState.HOMING_EXTENSION;
                }
                break;
            case HOMING_EXTENSION:
                if (extensionLimitSwitch.isPressed()) {
                    MotorUtility.reset(leaderExtensionMotor, followerExtensionMotor);
                    homingState = HomingState.HOMING_ROTATION;
                } else {
                    leaderExtensionMotor.setPower(EXTENSION_HOMING_POWER);
                    followerExtensionMotor.setPower(EXTENSION_HOMING_POWER);
                }
                break;
            case HOMING_ROTATION:
                if (rotationLimitSwitch.isPressed()) {
                    MotorUtility.reset(rotationMotor);
                    homingState = HomingState.COMPLETE;
                } else {
                    rotationMotor.setPower(ROTATION_HOMING_POWER);
                }
                break;
            case COMPLETE:
                armState = ArmState.AT_POS;
                rotationTargetPosition  = 0;
                extensionTargetPosition = 0;
                MotorUtility.reset(rotationMotor, leaderExtensionMotor, followerExtensionMotor);
                break;
        }
    }

    public void setTargetPosition(double extensionInches, double rotationDegrees) {
        if (armState == ArmState.HOMING) return;

        extensionInches = Range.clip(extensionInches, MIN_EXTENSION_INCHES, MAX_EXTENSION_INCHES);
        rotationDegrees = Range.clip(rotationDegrees, MIN_ROTATION_DEGREES, MAX_ROTATION_DEGREES);

        extensionTargetPosition = (int) ((extensionInches * EXTENSION_TICKS_PER_INCH) + 0.5);
        rotationTargetPosition  = (int) ((rotationDegrees * ROTATION_TICKS_PER_DEGREE) + 0.5);

        armState = ArmState.TO_POS;
    }

    public void openIntake() { intakeServo.setPosition(INTAKE_OPEN_POSITION); }

    public void closeIntake() { intakeServo.setPosition(INTAKE_CLOSED_POSITION); }

    public void zeroIntake() { intakeServo.setPosition(0.0); }

    // ---------------------------------------------------------------------------------------------
    // Getters
    // ---------------------------------------------------------------------------------------------

    public ArmState state() { return armState; }

    public int rotationPosition() { return rotationMotor.getCurrentPosition(); }

    public int extensionPosition() { return leaderExtensionMotor.getCurrentPosition(); }

    public double angleDegrees() { return rotationDegrees; }

    public double extensionInches() { return extensionInches + INTAKE_LENGTH_INCHES; }

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
        telemetry.addData("Rotation Limit Switch Pressed", rotationLimitSwitch.isPressed());
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
        telemetry.addData("Inches", extensionInches);
        telemetry.addData("Target Inches", targetExtensionInches());
        telemetry.addData("Displacement Inches", extensionDisplacementInches);
        telemetry.addData("Power", leaderExtensionMotor.getPower());
        telemetry.addData("At Position", extensionAtPosition());
        telemetry.addLine("----- Rotation -----");
        telemetry.addData("Position", rotationMotor.getCurrentPosition());
        telemetry.addData("Target Position", rotationTargetPosition);
        telemetry.addData("Degrees", angleDegrees());
        telemetry.addData("Target Degrees", targetAngleDegrees());
        telemetry.addData("Power", rotationMotor.getPower());
        telemetry.addData("At Position", rotationAtPosition());
    }

    public void debugCurrent() {
        double extensionCurrent = leaderExtensionMotor.getCurrent(CurrentUnit.AMPS)
                                + followerExtensionMotor.getCurrent(CurrentUnit.AMPS);
        double rotationCurrent  = rotationMotor.getCurrent(CurrentUnit.AMPS);

        telemetry.addData("Extension (AMPS)", extensionCurrent);
        telemetry.addData("Rotation (AMPS)", rotationCurrent);
        telemetry.addData("Total (AMPS)", extensionCurrent + rotationCurrent);
    }

    // ---------------------------------------------------------------------------------------------
    // State Enums
    // ---------------------------------------------------------------------------------------------

    public enum ArmState {
        HOMING,
        AT_POS,
        HOOKING_SEQUENCE,
        TO_POS
    }

    public enum HomingState {
        START,
        HOMING_EXTENSION,
        HOMING_ROTATION,
        COMPLETE
    }
}
