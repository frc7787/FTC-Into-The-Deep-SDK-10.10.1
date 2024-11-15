package org.firstinspires.ftc.teamcode.subsystems.arm;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.*;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utility.MotorUtility;

import static org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS;
import static org.firstinspires.ftc.teamcode.Constants.ArmConstants.*;

import java.util.ArrayList;

public final class Arm {

    private enum ArmState {
        TO_POS,
        AT_POS
    }

    // ---------------------------------------------------------------------------------------------
    // Global Variables
    // ---------------------------------------------------------------------------------------------

    // ---------- Hardware ---------- //

    private final RevTouchSensor frontRotationLimitSwitch,
                                 backRotationLimitSwitch;

    private final DcMotorImplEx rotationMotor,
                                extensionMotorOne,
                                extensionMotorTwo;

    // ---------- Configuration ---------- //

    private double maxExtensionPower,
                   maxRotationPower;

    // ---------- Global State ---------- //

    private ArmState armState;

    private boolean frontRotationLimitSwitchWasPressed,
                    backRotationLimitSwitchWasPressed;

    private int extensionTargetPosition,
                rotationTargetPosition;

    // ---------- Debug ---------- //

    private final Telemetry telemetry;

    private boolean debugCurrent;

    private ArrayList<Double> armCurrents,
                                    rotationCurrents,
                                    extensionCurrents,
                                    extensionMotorOneCurrents,
                                    extensionMotorTwoCurrents;

    private double armCurrent,
                   rotationCurrent,
                   extensionCurrent,
                   extensionMotorOneCurrent,
                   extensionMotorTwoCurrent;

    private double averageArmCurrent,
                   averageRotationCurrent,
                   averageExtensionCurrent,
                   averageExtensionMotorOneCurrent,
                   averageExtensionMotorTwoCurrent;

    private double peakArmCurrent,
            peakRotationCurrent,
            peakExtensionCurrent,
            peakExtensionMotorOneCurrent,
                   peakExtensionMotorTwoCurrent;

    // ---------------------------------------------------------------------------------------------
    // Constructors
    // ---------------------------------------------------------------------------------------------

    /**
     * Constructs a new arm subsystem
     * @param opMode The opMode you are creating the arm in ; to obtain the hardware map and
     *               telemetry.
     */
    public Arm(@NonNull OpMode opMode) {
        telemetry = opMode.telemetry;

        this.extensionMotorOne
                = opMode.hardwareMap.get(DcMotorImplEx.class, "extensionMotorOne");
        this.extensionMotorTwo
                = opMode.hardwareMap.get(DcMotorImplEx.class, "extensionMotorTwo");
        this.rotationMotor
                = opMode.hardwareMap.get(DcMotorImplEx.class, "rotationMotor");
        this.frontRotationLimitSwitch
                = opMode.hardwareMap.get(RevTouchSensor.class, "forwardRotationLimitSwitch");
        this.backRotationLimitSwitch
                = opMode.hardwareMap.get(RevTouchSensor.class, "reverseRotationLimitSwitch");

        MotorUtility.reset(extensionMotorOne, extensionMotorTwo, rotationMotor);

        extensionTargetPosition = 0;
        rotationTargetPosition  = 0;

        maxExtensionPower = DEFAULT_EXTENSION_MAX_POWER;
        maxRotationPower  = DEFAULT_ROTATION_MAX_POWER;

        initializeCurrentInformation();

        frontRotationLimitSwitchWasPressed = false;
        backRotationLimitSwitchWasPressed  = false;

        armState = ArmState.AT_POS;
    }

    /**
     * Constructs a new arm.
     * @param opMode The opMode you are constructing the arm in
     * @param maxExtensionPower The maximum power to supply the extension motors with while trying
     *                          to reach a target position
     * @param maxRotationPower The maximum power to supply the rotation motors with while trying
     *                         to reach a target position
     */
    public Arm(@NonNull OpMode opMode, double maxExtensionPower, double maxRotationPower) {
        telemetry = opMode.telemetry;

        this.extensionMotorOne
                = opMode.hardwareMap.get(DcMotorImplEx.class, "extensionMotorOne");
        this.extensionMotorTwo
                = opMode.hardwareMap.get(DcMotorImplEx.class, "extensionMotorTwo");
        this.rotationMotor
                = opMode.hardwareMap.get(DcMotorImplEx.class, "rotationMotor");
        this.frontRotationLimitSwitch
                = opMode.hardwareMap.get(RevTouchSensor.class, "forwardRotationLimitSwitch");
        this.backRotationLimitSwitch
                = opMode.hardwareMap.get(RevTouchSensor.class, "reverseRotationLimitSwitch");

        MotorUtility.reset(extensionMotorOne, extensionMotorTwo, rotationMotor);

        extensionTargetPosition = 0;
        rotationTargetPosition  = 0;

        this.maxExtensionPower = maxExtensionPower;
        this.maxRotationPower  = maxRotationPower;

        initializeCurrentInformation();

        frontRotationLimitSwitchWasPressed = false;
        backRotationLimitSwitchWasPressed  = false;
    }

    // ---------------------------------------------------------------------------------------------
    // Core
    // ---------------------------------------------------------------------------------------------

    /**
     * Method to update the state of the arm. This function should be called once every iteration
     * of the loop
     */
    public void update() {
        switch (armState) {
            case AT_POS:
                extensionMotorOne.setPower(0);
                extensionMotorTwo.setPower(0);
                rotationMotor.setPower(0);
                break;
            case TO_POS:
                rotate(rotationTargetPosition);
                extend(extensionTargetPosition);

                if (!rotationMotor.isBusy() && !extensionMotorOne.isBusy()) {
                    armState = ArmState.AT_POS;
                }

                break;
        }

        frontRotationLimitSwitchWasPressed = frontRotationLimitSwitch.isPressed();
        backRotationLimitSwitchWasPressed  = backRotationLimitSwitch.isPressed();
    }


    /**
     * Sets the target position of the arm
     * @param rotationTargetPosition The target position of the rotation motor
     * @param extensionTargetPosition The target position of the extension motor, note that this
     *                                only sets the target position of one of the motors, the second
     *                                motor copies the power of the first.
     */
    public void setTargetPosition(
            int rotationTargetPosition,
            int extensionTargetPosition
    ) {
        this.rotationTargetPosition  = Range.clip(rotationTargetPosition, 0, 9500);
        this.extensionTargetPosition = extensionTargetPosition;
        armState = ArmState.TO_POS;
    }

    /**
     * Sets the extension target position of the arm
     * @param extensionTargetPosition The target position of the extension motor
     */
    public void setExtensionTargetPosition(int extensionTargetPosition) {
        this.extensionTargetPosition = extensionTargetPosition;
        armState = ArmState.TO_POS;
    }

    /**
     * Sets the rotation target position of the arm
     * @param rotationTargetPosition The target position of the rotation motor
     */
    public void setRotationTargetPosition(int rotationTargetPosition) {
        this.rotationTargetPosition = Range.clip(rotationTargetPosition, 0, 9500);
        armState = ArmState.TO_POS;
    }

    /**
     * Sets the maximum power the arm can reach (between 0.0 & 1.0) when attempting to reach the
     * target position
     * @param rotationMaxPower The max power the rotation motor can reach
     * @param extensionMaxPower The max power the extension motors can reach
     */
    public void setMaxArmPower(
            double rotationMaxPower,
            double extensionMaxPower
    ) {
        this.maxRotationPower  = rotationMaxPower;
        this.maxExtensionPower = extensionMaxPower;
    }

    /**
     * Extends the arm to the target position at the specified power. Must be called continually
     * or the arm will keep extending until it physically cannot, which will probably break
     * something.
     * @param targetPosition The position to extend to
     * @param maxPower The maximum power to supply to extension motors while moving to the target
     *                 position
     */
    private void extend(int targetPosition, double maxPower) {
        extensionMotorOne.setTargetPosition(targetPosition);
        extensionMotorOne.setMode(RUN_TO_POSITION);
        extensionMotorOne.setPower(maxPower);
        extensionMotorTwo.setTargetPosition(targetPosition);
        extensionMotorTwo.setMode(RUN_TO_POSITION);
        extensionMotorTwo.setPower(maxPower);
    }

    /**
     * Extends the arm to the target position at the power defined by DEFAULT_EXTENSION_POWER.
     * Must be called continually or the arm will keep extending until it physically cannot,
     * which will probably break something.
     * @param targetPosition The position to extend to
     */
    private void extend(int targetPosition) {
        extend(targetPosition, DEFAULT_EXTENSION_MAX_POWER);
    }

    /**
     * Rotates the arm to the target position at the specified power. Must be called continually
     * or the arm will keep rotating until it physically cannot, which will probably break
     * something.
     * @param targetPosition The position to rotate to
     * @param maxPower The maximum power to supply the rotation motor while moving to the target
     *                 position.
     */
    private void rotate(int targetPosition, double maxPower) {
        rotationMotor.setTargetPosition(targetPosition);
        rotationMotor.setMode(RUN_TO_POSITION);
        rotationMotor.setPower(maxPower);
    }

    /**
     * Rotates the arm to the target position at the power defined by DEFAULT_ROTATION_POWER.
     * Must be called continually or the arm will keep rotating until it physically cannot,
     * which will probably break something.
     * @param targetPosition The position to rotate to
     */
    private void rotate(int targetPosition) {
        rotate(targetPosition, DEFAULT_ROTATION_MAX_POWER);
    }

    /**
     * Converts the input ticks, from the arm, to its rotation
     * @param ticks The position of the rotation motor
     * @return The angle of the rotation motor in degrees
     */
    private double rotationTicksToRadians(int ticks) {
        double degrees = ((double) ticks) / 55;
        return Math.toRadians(degrees);
    }

    // ---------------------------------------------------------------------------------------------
    // Telemetry
    // ---------------------------------------------------------------------------------------------

    /**
     * @return The current position of the leading extension motor
     */
    public int extensionPosition() {
        return extensionMotorOne.getCurrentPosition();
    }

    /**
     * @return The current position of the rotation motor
     */
    public int rotationPosition() {
        return rotationMotor.getCurrentPosition();
    }

    /**
     * @return The target position of the leading extension motor
     */
    public int extensionTargetPosition() {
        return extensionMotorOne.getTargetPosition();
    }

    public int rotationTargetPosition() {
        return rotationMotor.getTargetPosition();
    }

    /**
     * @return Whether either rotation limit switch is pressed
     */
    public boolean rotationLimitSwitchIsPressed() {
        return frontRotationLimitSwitch.isPressed() || backRotationLimitSwitch.isPressed();
    }

    // ---------------------------------------------------------------------------------------------
    // Debug
    // ---------------------------------------------------------------------------------------------

    /**
     * Sets the power of the arm motors, should only be used for debugging purposes
     * @param extensionPower The power to supply the extension motors
     * @param rotationPower The power to supply the rotation motors
     */
    public void debugSetArmPower(double extensionPower, double rotationPower) {
        extensionMotorOne.setPower(extensionPower);
        extensionMotorTwo.setPower(extensionPower);
        rotationMotor.setPower(rotationPower);
    }

    /**
     * Displays full debug information about the arm
     */
    public void debugAll() {
       debugGlobal();
       debugRotation();
       debugExtension();
    }

    /**
     * Displays debug information about the global state of the arm. The information displayed
     * includes the following:
     * <ul>
     *     <li>Arm State</li>
     *     <li>Homing State</li>
     *     <li>The states of both of the limit switches</li>
     * </ul>
     */
    public void debugGlobal() {
        telemetry.addLine("----- Global State -----");
        telemetry.addData(
                "Front Rotation Limit Switch Pressed", frontRotationLimitSwitch.isPressed());
        telemetry.addData(
                "Front Rotation Limit Switch Was Pressed", frontRotationLimitSwitchWasPressed);
        telemetry.addData(
                "Back Rotation Limit Switch Pressed", backRotationLimitSwitch.isPressed());
        telemetry.addData(
                "Back Rotation Limit Switch Was Pressed", backRotationLimitSwitchWasPressed);
        telemetry.addData("Arm State", armState);
    }

    /**
     * Displays debug information about the rotation motor. The information displayed is as follows:
     * <ul>
     *     <li>The power</li>
     *     <li>The current and target position</li>
     *     <li>The current draw, in amps</li>
     *     <li>The direction </li>
     *     <li>The zero power behaviour </li>
     *     <li>The run mode </li>
     * </ul>
     */
    public void debugRotation() {
        telemetry.addLine("----- Rotation Motor -----");
        telemetry.addData("Power", rotationMotor.getPower());
        telemetry.addData("Position", rotationMotor.getCurrentPosition());
        telemetry.addData("Target Position (Subsystem)", rotationTargetPosition);
        telemetry.addData("Target Position (Motor)", rotationMotor.getTargetPosition());
        telemetry.addData("Current (AMPS)", rotationMotor.getCurrent(AMPS));
        telemetry.addData("Velocity", rotationMotor.getVelocity());
        telemetry.addData("Direction", rotationMotor.getDirection());
        telemetry.addData("Zero Power Behaviour", rotationMotor.getZeroPowerBehavior());
        telemetry.addData("Run Mode", rotationMotor.getMode());
    }

    /**
     * Displays debug information about extension motor one. The information displayed is as
     * follows:
     * <ul>
     *     <li>The power</li>
     *     <li>The current and target position</li>
     *     <li>The current draw, in amps</li>
     *     <li>The direction </li>
     *     <li>The zero power behaviour </li>
     *     <li>The run mode </li>
     * </ul>
     */
    public void debugExtensionMotorOne() {
        telemetry.addLine("----- Extension Motor One -----");
        telemetry.addData("Power", extensionMotorOne.getPower());
        telemetry.addData("Position", extensionMotorOne.getCurrentPosition());
        telemetry.addData("Target Position (Subsystem)", extensionTargetPosition);
        telemetry.addData("Target Position (Motor)", extensionMotorOne.getTargetPosition());
        telemetry.addData("Current (AMPS)", extensionMotorOne.getCurrent(AMPS));
        telemetry.addData("Direction", extensionMotorOne.getDirection());
        telemetry.addData("Zero Power Behaviour", extensionMotorOne.getZeroPowerBehavior());
        telemetry.addData("Run Mode", extensionMotorOne.getMode());
    }

    /**
     * Displays debug information about extension motor two. The information displayed is as
     * follows:
     * <ul>
     *     <li>The power</li>
     *     <li>The current and target position</li>
     *     <li>The current draw, in amps</li>
     *     <li>The direction </li>
     *     <li>The zero power behaviour </li>
     *     <li>The run mode </li>
     * </ul>
     */
    public void debugExtensionMotorTwo() {
        telemetry.addLine("----- Extension Motor Two -----");
        telemetry.addData("Power", extensionMotorTwo.getPower());
        telemetry.addData("Position", extensionMotorTwo.getCurrentPosition());
        telemetry.addData("Target Position (Subsystem)", extensionTargetPosition);
        telemetry.addData("Target Position", extensionMotorTwo.getTargetPosition());
        telemetry.addData("Current (AMPS)", extensionMotorTwo.getCurrent(AMPS));
        telemetry.addData("Direction", extensionMotorTwo.getDirection());
        telemetry.addData("Zero Power Behaviour", extensionMotorTwo.getZeroPowerBehavior());
        telemetry.addData("Run Mode", extensionMotorOne.getMode());
    }

    /**
     * Displays debug information about both extension motors. The information displayed is as
     * follows:
     * <ul>
     *     <li>The power</li>
     *     <li>The current and target position</li>
     *     <li>The current draw, in amps</li>
     *     <li>The direction </li>
     *     <li>The zero power behaviour </li>
     *     <li>The run mode </li>
     * </ul>
     */
    public void debugExtension() {
        debugExtensionMotorOne();
        debugExtensionMotorTwo();
    }

    private void initializeCurrentInformation() {
        rotationCurrents          = new ArrayList<>();
        extensionMotorOneCurrents = new ArrayList<>();
        extensionMotorTwoCurrents = new ArrayList<>();
        extensionCurrents         = new ArrayList<>();
        armCurrents               = new ArrayList<>();

        rotationCurrent          = 0;
        extensionMotorOneCurrent = 0;
        extensionMotorTwoCurrent = 0;
        extensionCurrent         = 0;
        armCurrent               = 0;

        peakRotationCurrent          = 0;
        peakExtensionMotorOneCurrent = 0;
        peakExtensionMotorTwoCurrent = 0;
        peakExtensionCurrent         = 0;
        peakArmCurrent               = 0;

        averageRotationCurrent          = 0;
        averageExtensionMotorOneCurrent = 0;
        averageExtensionMotorTwoCurrent = 0;
        averageExtensionCurrent         = 0;
        averageArmCurrent               = 0;
    }

    /**
     * Records information about the current draw of the arm
     */
    private void recordCurrentInformation() {
        rotationCurrent          = rotationMotor.getCurrent(AMPS);
        extensionMotorOneCurrent = extensionMotorOne.getCurrent(AMPS);
        extensionMotorTwoCurrent = extensionMotorTwo.getCurrent(AMPS);
        extensionCurrent         = extensionMotorOneCurrent + extensionMotorTwoCurrent;
        armCurrent               = extensionCurrent + rotationCurrent;

        if (rotationCurrent > peakRotationCurrent) {
            peakRotationCurrent = rotationCurrent;
        }

        if (extensionMotorOneCurrent > peakExtensionMotorOneCurrent) {
            peakExtensionMotorOneCurrent = extensionMotorOneCurrent;
        }

        if (extensionMotorTwoCurrent > peakExtensionMotorTwoCurrent) {
            peakExtensionMotorTwoCurrent = extensionMotorTwoCurrent;
        }

        if (extensionCurrent > peakExtensionCurrent) {
            peakArmCurrent = extensionCurrent;
        }

        if (armCurrent > peakArmCurrent) {
            peakArmCurrent = armCurrent;
        }

        if (rotationCurrent > 0.0) {
            rotationCurrents.add(rotationCurrent);
        }

        if (extensionMotorOneCurrent > 0.0) {
            extensionMotorOneCurrents.add(extensionMotorOneCurrent);
        }

        if (extensionMotorTwoCurrent > 0.0) {
            extensionMotorTwoCurrents.add(extensionMotorTwoCurrent);
        }

        if (extensionCurrent > 0.0) {
            extensionCurrents.add(extensionCurrent);
        }

        if (armCurrent > 0.0) {
            armCurrents.add(armCurrent);
        }

        if (!rotationCurrents.isEmpty()) {
            double rotationCurrentSum = 0;
            for (Double rotationCurrent : rotationCurrents) {
                rotationCurrentSum += rotationCurrent;
            }
            averageRotationCurrent = rotationCurrentSum / rotationCurrents.size();
        }

        if (!extensionMotorOneCurrents.isEmpty()) {
            double extensionMotorOneCurrentSum = 0;
            for (Double extensionMotorOneCurrent : extensionMotorOneCurrents) {
                extensionMotorOneCurrentSum += extensionMotorOneCurrent;
            }
            averageExtensionMotorOneCurrent
                    = extensionMotorOneCurrentSum / extensionMotorOneCurrents.size();
        }

        if (!extensionMotorTwoCurrents.isEmpty()) {
            double extensionMotorTwoCurrentSum = 0;
            for (Double extensionMotorTwoCurrent : extensionMotorTwoCurrents) {
                extensionMotorTwoCurrentSum += extensionMotorTwoCurrent;
            }
            averageExtensionMotorTwoCurrent
                    = extensionMotorTwoCurrentSum / extensionMotorTwoCurrents.size();
        }

        if (!extensionCurrents.isEmpty()) {
            double extensionCurrentSum = 0;
            for (Double extensionCurrent : extensionCurrents) {
                extensionCurrentSum += extensionCurrent;
            }
            averageExtensionCurrent = extensionCurrentSum / extensionCurrents.size();
        }

        if (!armCurrents.isEmpty()) {
            double armCurrentSum = 0;
            for (Double armCurrent: armCurrents) {
                armCurrentSum += armCurrent;
            }
            averageArmCurrent = armCurrentSum / armCurrents.size();
        }
    }

    /**
     * Displays information about the current, peak, and average current draw of the rotation
     * motor. All current measurements are displays in amps.
     */
    public void debugRotationCurrent() {
        if (!debugCurrent) {
            debugCurrent = true;
        } else {
            telemetry.addLine("----- Rotation Current (Amps) -----");
            telemetry.addData("Current", rotationCurrent);
            telemetry.addData("Peak", peakRotationCurrent);
            telemetry.addData("Average", averageRotationCurrent);
        }
    }

    /**
     * Displays information about the current, peak, and average current draw of the lead extension
     * motor. All current measurements are displayed in amps.
     */
    public void debugExtensionMotorOneCurrent() {
        if (!debugCurrent) {
            debugCurrent = true;
        } else {
            telemetry.addLine("----- Extension Motor One Current (Amps) -----");
            telemetry.addData("Current", extensionMotorOneCurrent);
            telemetry.addData("Peak", peakExtensionMotorOneCurrent);
            telemetry.addData("Average", averageExtensionMotorOneCurrent);
        }
    }

    /**
     * Displays information about the current, peak, and average current draw of the following
     * extension motor. All current measurements are displayed in amps.
     */
    public void debugExtensionMotorTwoCurrent() {
        if (!debugCurrent) {
            debugCurrent = true;
        } else {
            telemetry.addLine("----- Extension Motor Two Current (Amps) -----");
            telemetry.addData("Current", extensionMotorTwoCurrent);
            telemetry.addData("Peak", peakExtensionMotorTwoCurrent);
            telemetry.addData("Average", averageExtensionMotorTwoCurrent);
        }
    }

    /**
     * Displays information about the current, peak, and average current draw of the extension
     * motors. All current measurements are displayed in amps.
     */
    public void debugExtensionCurrent() {
        if (!debugCurrent) {
            debugCurrent = true;
        } else {
            telemetry.addLine("----- Extension Current (Amps) -----");
            telemetry.addData("Current", extensionCurrent);
            telemetry.addData("Peak", peakExtensionCurrent);
            telemetry.addData("Average", averageExtensionCurrent);
        }
    }

    /**
     * Displays information about the current, peak, and average current draw of the arm motors.
     * All current measurements are displayed in amps.
     */
    public void debugArmCurrent() {
        if (!debugCurrent) {
            debugCurrent = true;
        } else {
            telemetry.addLine("----- Arm Current (Amps) ------");
            telemetry.addData("Current", armCurrent);
            telemetry.addData("Peak", peakArmCurrent);
            telemetry.addData("Average", averageArmCurrent);
        }
    }


}
