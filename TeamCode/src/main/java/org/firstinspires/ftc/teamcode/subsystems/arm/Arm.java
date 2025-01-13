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
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.utility.*;
import static org.firstinspires.ftc.teamcode.subsystems.arm.ArmConstants.*;
import static org.firstinspires.ftc.teamcode.subsystems.arm.ArmConversions.*;

public class Arm {
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
    private double vExtensionTargetInches, hExtensionTargetInches;

    private double maxSpeed;

    private boolean isFirstManualControlIteration;
    private ElapsedTime manualControlTimer;

    // ---------------------------------------------------------------------------------------------
    // Debug
    // ---------------------------------------------------------------------------------------------

    private final Telemetry telemetry;

    // ---------------------------------------------------------------------------------------------
    // Controllers
    // ---------------------------------------------------------------------------------------------

    private final PIDController extensionController
            = new PIDController(0.00135, 0, 0.0001);

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

        maxSpeed = MAX_EXTENSION_POWER;

        hExtensionTargetInches = START_POSITION_XY[0];
        vExtensionTargetInches = START_POSITION_XY[1];

        armState = ArmState.HOMING;
        homingState = HomingState.START;
        isFirstManualControlIteration = true;
        manualControlTimer = new ElapsedTime();
        manualControlTimer.reset();
    }

    // ---------------------------------------------------------------------------------------------
    // Core
    // ---------------------------------------------------------------------------------------------

    public void update() {
        rotationPosition = rotationMotor.getCurrentPosition();
        extensionPosition = leaderExtensionMotor.getCurrentPosition();

        switch (armState) {
            case HOMING:
                home();
                break;
            case NORMAL:
                double rotationPower
                        = rotationController.calculate(rotationPosition, rotationTargetPosition);
                rotationPower = Range.clip(rotationPower, MIN_ROTATION_POWER, MAX_ROTATION_POWER);
                double extensionPower
                        = extensionController.calculate(extensionPosition, extensionTargetPosition);
                extensionPower = Range.clip(extensionPower, MIN_EXTENSION_POWER, maxSpeed);

                if (extensionTargetPosition <= 0 && extensionLimitSwitch.isPressed()) {
                    extensionPower = 0.0;
                }

                if (rotationTargetPosition <= 0 && frontRotationLimitSwitch.isPressed()) {
                    rotationPower = 0.0;
                }

                if (backRotationLimitSwitch.isPressed() && rotationPower >= 0.0) {
                    rotationPower = 0.0;
                }

                if (-0.1 < rotationPower && rotationPower < 0.1 && rotationAtPosition()) {
                    rotationPower = 0.0;
                }

                if (-0.2 < extensionPower && extensionPower < 0.15 && extensionAtPosition()) {
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

    public void stop() {
        rotationMotor.setPower(0);
        followerExtensionMotor.setPower(0);
        leaderExtensionMotor.setPower(0);
    }

    public void manualControlPolar(double thetaInput, double radiusInput, double speed) {

    }

    public void manualControlCartesian(double xInput, double yInput, double speed) {
        hExtensionTargetInches += (xInput * manualControlTimer.seconds() * speed);
        vExtensionTargetInches += (yInput * manualControlTimer.seconds() * speed);
        manualControlTimer.reset();
        if (hExtensionTargetInches > 30) hExtensionTargetInches = 30;
        if (vExtensionTargetInches < -5) vExtensionTargetInches = -5;
        double[] polarCoordinates = cartesianToPolar(hExtensionTargetInches, vExtensionTargetInches);
        rotationTargetPosition = rotationDegreesToTicks(polarCoordinates[0]);
        extensionTargetPosition = extensionInchesToTicks(polarCoordinates[1]);
    }

    public void manualControlCartesian(double xInput, double yInput) {
        manualControlCartesian(xInput, yInput, DEFAULT_MANUAL_SPEED);
    }

    public void resetManualControl() {
        isFirstManualControlIteration = true;
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

    public void setTargetPositionInches(double hExtensionTargetInches, double vExtensionTargetInches) {
        if (hExtensionTargetInches >= 30) hExtensionTargetInches = 30;
        this.hExtensionTargetInches = hExtensionTargetInches;
        this.vExtensionTargetInches = vExtensionTargetInches;
        double[] polarCoordinates = cartesianToPolar(hExtensionTargetInches,vExtensionTargetInches);
        rotationTargetPosition = rotationDegreesToTicks(polarCoordinates[0]);
        extensionTargetPosition = extensionInchesToTicks(polarCoordinates[1]);
    }

    public void setTargetPositionInchesRobotCentric(double hExtensionTargetInches, double vExtensionTargetInches) {
        hExtensionTargetInches += ROTATION_X_OFFSET_INCHES;
        vExtensionTargetInches += ROTATION_Y_OFFSET_INCHES;
        setTargetPositionInches(hExtensionTargetInches, vExtensionTargetInches);
    }

    public void setExtensionTargetPosition(double extensionInches) {
        extensionInches = Math.min(25, Math.max(extensionInches, 0));
        extensionTargetPosition = (int) (extensionInches * EXTENSION_TICKS_PER_INCH);
    }

    public void setIntakePosition(double position) {
        intakeServo.setPosition(position);
    }

    public void zeroIntake() { intakeServo.setPosition(0.0); }

    public void setMaxSpeed(double speed) {
        maxSpeed = Range.clip(speed, 0.0, 1.0);
    }

    // ---------------------------------------------------------------------------------------------
    // Getters
    // ---------------------------------------------------------------------------------------------

    public ArmState state() { return armState; }

    public double hExtensionTargetInches() {
        return hExtensionTargetInches;
    }

    public double vExtensionTargetInches() {
        return vExtensionTargetInches;
    }

    public int rotationPosition() { return rotationMotor.getCurrentPosition(); }

    public int extensionPosition() { return leaderExtensionMotor.getCurrentPosition(); }

    public double degrees() { return rotationMotor.getCurrentPosition() / ROTATION_TICKS_PER_DEGREE ; }

    public double inches() {
        return (leaderExtensionMotor.getCurrentPosition() / EXTENSION_TICKS_PER_INCH);
    }

    public double intakePosition() {
        return intakeServo.getPosition();
    }

    public int rotationTargetPosition() { return rotationTargetPosition; }

    public int extensionTargetPosition() { return extensionTargetPosition; }

    public double targetAngleDegrees() {
        return rotationTargetPosition / ROTATION_TICKS_PER_DEGREE;
    }

    public double targetInches() {
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
        return extensionPosition <= EXTENSION_POSITION_THRESHOLD + extensionTargetPosition
                && extensionPosition >= -EXTENSION_NEGATIVE_THRESHOLD + extensionTargetPosition;
    }

    public boolean isAtPosition() { return rotationAtPosition() && extensionAtPosition(); }

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
        telemetry.addData("Target Inches", targetInches());
        telemetry.addData("Power", leaderExtensionMotor.getPower());
        telemetry.addData("At Position", extensionAtPosition());
        telemetry.addLine("----- Rotation -----");
        telemetry.addData("Position", rotationMotor.getCurrentPosition());
        telemetry.addData("Target Position", rotationTargetPosition);
        telemetry.addData("Degrees", rotationMotor.getCurrentPosition() / ROTATION_TICKS_PER_DEGREE);
        telemetry.addData("Target Degrees", targetAngleDegrees());
        telemetry.addData("Power", rotationMotor.getPower());
        telemetry.addData("At Position", rotationAtPosition());
        telemetry.addData("Horizontal Target Inches", hExtensionTargetInches);
        telemetry.addData("Vertical Target Inches", vExtensionTargetInches);
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
