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
        if (debug) {

        }

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
     * Extends the arm to the target position at the specified power. Must be called continually
     * or the arm will keep extending until it physically cannot, which will probably break
     * something.
     * @param targetPosition The position to extend to
     * @param maxPower The maximum power to supply to extension motors while moving to the target
     *                 position
     */
    private void extend(int targetPosition, double maxPower) {
        int currentPosition = extensionMotorOne.getCurrentPosition();
        double power = extensionController.calculate(currentPosition, targetPosition);
        power = Range.clip(power, -maxPower, maxPower);
        extensionMotorOne.setPower(power);
        extensionMotorTwo.setPower(power);
    }

    /**
     * Extends the arm to the target position at the power defined by DEFAULT_EXTENSION_POWER.
     * Must be called continually or the arm will keep extending until it physically cannot,
     * which will probably break something.
     * @param targetPosition The position to extend to
     */
    private void extend(int targetPosition) {
        extend(targetPosition, DEFAULT_EXTENSION_POWER);
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
        int currentPosition = rotationMotor.getCurrentPosition();
        double power = rotationController.calculate(currentPosition, targetPosition);
        power = Range.clip(power, -maxPower, maxPower);
        rotationMotor.setPower(power);
    }

    /**
     * Rotates the arm to the target position at the power defined by DEFAULT_ROTATION_POWER.
     * Must be called continually or the arm will keep rotating until it physically cannot,
     * which will probably break something.
     * @param targetPosition The position to rotate to
     */
    private void rotate(int targetPosition) {
        rotate(targetPosition, DEFAULT_ROTATION_POWER);
    }

    /**
     * Runs the homing sequence of the arm. This function is not blocking and should be called
     * continually until it is finished. Once it is finished it will set the armState variable to
     * AT_POS from which normal control of the robot can be resumed.
     */
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

    /**
     * @return The current position of extensionMotorOne
     */
    public int extensionCurrentPosition() {
        return extensionMotorOne.getCurrentPosition();
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
        telemetry.addLine("----- Global -----");
        telemetry.addData("Arm State", armState);
        telemetry.addData("Homing State", homingState);
        telemetry.addData("Rotation Limit Switch Pressed", rotationLimitSwitch.isPressed());
        telemetry.addData("Extension Limit Switch Pressed", extensionLimitSwitch.isPressed());
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

    /**
     * Records information about the current draw of the arm
     */
    private void recordCurrentInformation() {

    }

    /**
     * Displays information about the current draw, in amps, of the motors. The information
     * displayed is as follows:
     * <ul>
     *     <li>The combined current of the extension motors</li>
     *     <li>The current of the rotation motor</li>
     *     <li>The maximum combined current of the extension motors</li>
     *     <li>The maximum current of the rotation motors</li>
     * </ul>
     */
    public void debugCurrent() {
        if (!debugCurrent) {
            debugCurrent = true;
            return;
        }

        double extensionMotorOneCurrentAmps = extensionMotorOne.getCurrent(AMPS);
        double extensionMotorTwoCurrentAmps = extensionMotorTwo.getCurrent(AMPS);
        double extensionCurrentAmps =
                extensionMotorOneCurrentAmps + extensionMotorTwoCurrentAmps;

        telemetry.addLine("All Current Measurements Are In Amps");
        telemetry.addData("Extension Motor One Current", extensionMotorOneCurrentAmps);
        telemetry.addData("Extension Motor One Peak Current", extensionMotorTwoCurrentAmps);
        telemetry.addLine();
        telemetry.addData(
                "Extension Motor One Average Current", averageExtensionMotorOneCurrentAmps);
        telemetry.addData("Extension Motor Two Current", extensionMotorTwo.getCurrent(AMPS));
        telemetry.addData("Extension Motor Two Peak Current", peakExtensionMotorTwoCurrentAmps);
        telemetry.addLine();
        telemetry.addData(
                "Extension Motor Two Average Current", averageExtensionMotorTwoCurrentAmps);
        telemetry.addData("Extension Current", extensionCurrentAmps);
        telemetry.addData("Peak Extension Current", peakExtensionCurrentAmps);
        telemetry.addData("Average Extension Current", averageExtensionCurrentAmps);
        telemetry.addLine();
        telemetry.addData("Rotation Motor Current ", rotationMotor.getCurrent(AMPS));
        telemetry.addData("Peak Rotation Motor Current", peakRotationMotorCurrentAmps);
        telemetry.addData("Average Rotation Motor Current", averageRotationMotorCurrentAmps);
    }

    /**
     * Converts the input ticks, from the arm, to its rotation
     * @param ticks The position of the rotation motor
     * @return The angle of the rotation motor in degrees
     */
    private double rotationTicksToRadians(int ticks) {
       return ticks;
    }
}
