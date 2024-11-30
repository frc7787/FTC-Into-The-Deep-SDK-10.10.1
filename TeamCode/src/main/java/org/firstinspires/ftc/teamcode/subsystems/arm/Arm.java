package org.firstinspires.ftc.teamcode.subsystems.arm;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utility.*;

// TODO -- Manual Control
// TODO -- Automatic Hook Sequence
// TODO -- Extension Limit
// TODO -- Clean Up Code
// TODO -- Measure Servo Positions

public class Arm {
    // ---------------------------------------------------------------------------------------------
    // Configuration
    // ---------------------------------------------------------------------------------------------

    // Physical Properties
    private final double INTAKE_LENGTH_INCHES      = 4.0;
    private final double ARM_STARTING_ANGLE_OFFSET = 2.0;
    private final double EXTENSION_TICKS_PER_INCH  = 53.8;
    private final double ROTATION_TICKS_PER_DEGREE = 26.3;
    private final double ROTATION_OFFSET_FROM_FRONT_INCHES = 9.75;

    // Physical minimum and maximum extension and rotation of the arm
    private final int MIN_ROTATION_DEGREES = 0;
    private final int MAX_ROTATION_DEGREES = 120;
    private final int MIN_EXTENSION_INCHES = 0;
    private final int MAX_EXTENSION_INCHES = 40;

    private final double MIN_ROTATION_TICKS = MIN_ROTATION_DEGREES * ROTATION_TICKS_PER_DEGREE;
    private final double MAX_ROTATION_TICKS = MAX_ROTATION_DEGREES * ROTATION_TICKS_PER_DEGREE;

    private final double MIN_EXTENSION_TICKS = MIN_EXTENSION_INCHES * EXTENSION_TICKS_PER_INCH;
    private final double MAX_EXTENSION_TICKS = MAX_EXTENSION_INCHES * EXTENSION_TICKS_PER_INCH;

    // Software limit for the extension of the arm
    private final double FORWARD_EXTENSION_LIMIT_INCHES = 60;
    private final double ROTATION_LIMIT_DEGREES         = 95;

    private final double MAX_EXTENSION_POWER = 0.8;
    private final double MAX_ROTATION_POWER = 1.0;

    private final double EXTENSION_HOMING_POWER = -0.8;
    private final double ROTATION_HOMING_POWER  = -1.0;

    private final int EXTENSION_POSITION_THRESHOLD = 10;
    private final int ROTATION_POSITION_THRESHOLD  = 10;

    private final double INTAKE_OPEN_POSITION   = 0.0;
    private final double INTAKE_CLOSED_POSITION = 0.3;

    // ---------------------------------------------------------------------------------------------
    // Hardware
    // ---------------------------------------------------------------------------------------------

    private final DcMotorImplEx extensionMotorOne,
                                extensionMotorTwo,
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

    // --------------------
    // Automatic Hooking State
    // --------------------

    private int hookingTargetPosition;
    private final ElapsedTime hookingServoTimer;
    private HookingState hookingState;
    private boolean isFirstReleasingIteration;
    private double rotationDegreesOnComplete;
    private double extensionInchesOnComplete;

    // ---------------------------------------------------------------------------------------------
    // Debug
    // ---------------------------------------------------------------------------------------------

    private final Telemetry telemetry;

    // ---------------------------------------------------------------------------------------------
    // Controllers
    // ---------------------------------------------------------------------------------------------

    private final PIDController extensionController
            = new PIDController(0.014, 0, 0);

    private final PIDController rotationController
            = new PIDController(0.008, 0, 0);

    // ---------------------------------------------------------------------------------------------
    // Construction
    // ---------------------------------------------------------------------------------------------

    public Arm(@NonNull OpMode opMode) {
        telemetry = opMode.telemetry;

        HardwareMap hardwareMap = opMode.hardwareMap;

        extensionMotorOne = hardwareMap.get(DcMotorImplEx.class, "extensionMotorOne");
        extensionMotorTwo = hardwareMap.get(DcMotorImplEx.class, "extensionMotorTwo");
        rotationMotor     = hardwareMap.get(DcMotorImplEx.class, "rotationMotor");

        MotorUtility.setZeroPowerBehaviours(
                BRAKE, extensionMotorOne, extensionMotorTwo, rotationMotor);
        MotorUtility.reset(extensionMotorOne, extensionMotorTwo, rotationMotor);

        rotationLimitSwitch = hardwareMap.get(RevTouchSensor.class, "rotationLimitSwitch");
        extensionLimitSwitch = hardwareMap.get(RevTouchSensor.class, "extensionLimitSwitch");

        intakeServo = hardwareMap.get(Servo.class, "intakeServo");
        intakeServo.setPosition(0.0);

        extensionTargetPosition = 0;
        rotationTargetPosition  = 0;

        hookingTargetPosition   = 0;
        hookingServoTimer = new ElapsedTime();
        hookingServoTimer.reset();
        hookingState = HookingState.START;
        isFirstReleasingIteration = true;

        armState    = ArmState.HOMING;
        homingState = HomingState.START;
    }

    // ---------------------------------------------------------------------------------------------
    // Core
    // ---------------------------------------------------------------------------------------------

    public void update() {
        int rotationPosition  = rotationMotor.getCurrentPosition();
        int extensionPosition = extensionMotorOne.getCurrentPosition();

        // If we are trying to retract all the way and we hit the limit switch we reset
        if (extensionTargetPosition == 0 && extensionLimitSwitch.isPressed()) {
            MotorUtility.reset(extensionMotorOne, extensionMotorOne);
        }

        // If we are trying to rotate to the ground and we hit the limit switch we reset
        if (rotationTargetPosition == 0 && rotationLimitSwitch.isPressed()) {
            MotorUtility.reset(rotationMotor);
        }

        switch (armState) {
            case HOMING:
                home();
                break;
            case AT_POS:
                extensionMotorOne.setPower(0.0);
                extensionMotorTwo.setPower(0.0);
                rotationMotor.setPower(0.0);
                break;
            case HOOKING_SEQUENCE:
                break;
            case TO_POS:
                int rotationTargetPosition  = this.rotationTargetPosition;
                int extensionTargetPosition = this.extensionTargetPosition;

                // Calculate the extension power based on the error
                double extensionPower
                        = extensionController.calculate(extensionPosition, extensionTargetPosition);
                extensionPower
                        = Range.clip(extensionPower, -MAX_EXTENSION_POWER, MAX_EXTENSION_POWER);
                extensionMotorOne.setPower(extensionPower);
                extensionMotorTwo.setPower(extensionPower);

                // Calculate the rotation power based on the error
                double rotationPower
                        = rotationController.calculate(rotationPosition, rotationTargetPosition);
                rotationPower
                        = Range.clip(rotationPower, -MAX_ROTATION_POWER, MAX_ROTATION_POWER);

                rotationMotor.setPower(rotationPower);

                // Check to see if we are at position
                if (Math.abs(rotationPosition - rotationTargetPosition) <= ROTATION_POSITION_THRESHOLD
                    && Math.abs(extensionPosition - extensionTargetPosition) <= EXTENSION_POSITION_THRESHOLD) {
                    armState = ArmState.AT_POS;
                }
                break;
        }
    }

    private void home() {
        switch (homingState) {
            case START:
                if (extensionLimitSwitch.isPressed() && rotationLimitSwitch.isPressed()) {
                    MotorUtility.reset(extensionMotorOne, extensionMotorTwo, rotationMotor);
                    homingState = HomingState.COMPLETE;
                } else {
                    homingState = HomingState.HOMING_EXTENSION;
                }
                break;
            case HOMING_EXTENSION:
                if (extensionLimitSwitch.isPressed()) {
                    MotorUtility.reset(extensionMotorOne, extensionMotorTwo);
                    homingState = HomingState.HOMING_ROTATION;
                } else {
                    extensionMotorOne.setPower(EXTENSION_HOMING_POWER);
                    extensionMotorTwo.setPower(EXTENSION_HOMING_POWER);
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
                MotorUtility.reset(rotationMotor, extensionMotorOne, extensionMotorTwo);
                break;
        }
    }

    public void setTargetPositionTicks(int extensionTicks, int rotationTicks) {
        double extensionInches = ((double) extensionTicks / EXTENSION_TICKS_PER_INCH);
        double rotationDegrees = (double) rotationTicks / ROTATION_TICKS_PER_DEGREE;

        setTargetPosition(extensionInches, rotationDegrees);
    }

    public void setTargetPosition(double extensionInches, double rotationDegrees) {
        if (armState == ArmState.HOMING) return;

        extensionInches = Range.clip(extensionInches, MIN_EXTENSION_INCHES, MAX_EXTENSION_INCHES)
                        - INTAKE_LENGTH_INCHES;
        if (extensionInches < 0) extensionInches = 0;
        rotationDegrees = Range.clip(rotationDegrees, MIN_ROTATION_DEGREES, MAX_ROTATION_DEGREES)
                        - ARM_STARTING_ANGLE_OFFSET;
        if (rotationDegrees < 0) rotationDegrees = 0;

        double extensionDisplacementInches = (extensionInches - ROTATION_OFFSET_FROM_FRONT_INCHES)
                                           * StrictMath.cos(rotationDegrees);

        if (extensionDisplacementInches >= FORWARD_EXTENSION_LIMIT_INCHES) {
            extensionInches = FORWARD_EXTENSION_LIMIT_INCHES / StrictMath.cos(rotationDegrees);
        }

        extensionTargetPosition = (int) (extensionInches * EXTENSION_TICKS_PER_INCH);
        if (extensionTargetPosition <= 0) extensionTargetPosition = 0;
        rotationTargetPosition  = (int) (rotationDegrees * ROTATION_TICKS_PER_DEGREE);

        armState = ArmState.TO_POS;
    }

    public void openIntake() { intakeServo.setPosition(INTAKE_OPEN_POSITION); }

    public void closeIntake() { intakeServo.setPosition(INTAKE_CLOSED_POSITION); }

    public void homeIntake() { intakeServo.setPosition(0.0); }

    // ---------------------------------------------------------------------------------------------
    // Getters
    // ---------------------------------------------------------------------------------------------

    public ArmState state() { return armState; }

    public int rotationPosition() { return rotationMotor.getCurrentPosition(); }

    public int extensionPosition() { return extensionMotorOne.getCurrentPosition(); }

    public double rotationAngle() {
        return (double) rotationMotor.getCurrentPosition() / ROTATION_TICKS_PER_DEGREE;
    }

    public double extensionInches() {
        return (extensionMotorOne.getCurrentPosition() / EXTENSION_TICKS_PER_INCH)
                + INTAKE_LENGTH_INCHES;
    }

    public int rotationTargetPosition() { return rotationTargetPosition; }

    public int extensionTargetPosition() { return extensionTargetPosition; }

    public double rotationTargetAngle() {
        return rotationTargetPosition / ROTATION_TICKS_PER_DEGREE;
    }

    public double extensionTargetInches() {
        return (extensionTargetPosition / EXTENSION_TICKS_PER_INCH);
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
        extensionMotorOne.setPower(extensionPower);
        extensionMotorTwo.setPower(extensionPower);
    }

    public void debugPosition() {
        int extensionPosition = extensionMotorOne.getCurrentPosition();
        int rotationPosition  = rotationMotor.getCurrentPosition();

        telemetry.addLine("----- Extension -----");
        telemetry.addData("Position", extensionPosition);
        telemetry.addData("Target Position", extensionTargetPosition);
        telemetry.addData("Inches", extensionInches());
        telemetry.addData("Target Inches", extensionTargetInches());
        telemetry.addData("Power", extensionMotorOne.getPower());
        telemetry.addLine("----- Rotation -----");
        telemetry.addData("Position", rotationPosition);
        telemetry.addData("Target Position", rotationTargetPosition);
        telemetry.addData("Degrees", rotationAngle());
        telemetry.addData("Target Degrees", rotationTargetAngle());
        telemetry.addData("Power", rotationMotor.getPower());
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

    private enum HookingState {
        START,
        RETRACTING,
        RELEASING,
        COMPLETE
    }
}
