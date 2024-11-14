package org.firstinspires.ftc.teamcode.subsystems.mecanumdrive;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.*;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.*;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.*;
import static org.firstinspires.ftc.teamcode.Constants.DrivebaseConstants.*;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utility.MotorUtility;
import org.firstinspires.ftc.teamcode.utility.PIDController;

public final class MecanumDriveSubsystem extends SubsystemBase {
    public enum DriveMode {
        ROBOT_CENTRIC,
        FIELD_CENTRIC
    }

    @NonNull
    private DriveMode driveMode;

    private boolean shouldHoldPosition;

    private double xTargetPosition, yTargetPosition;

    private boolean hasRecordedXPosition, hasRecordedYPosition;

    private final PIDController drivePIDController = new PIDController(0.007, 0.0, 0.00007);
    private final PIDController strafePIDController = new PIDController(0.009, 0.0, 0.00009);

    private double drive, strafe, turn;

    @NonNull
    private final DcMotorImplEx frontLeftMotor,
                                frontRightMotor,
                                backLeftMotor,
                                backRightMotor;


    @NonNull
    private final Telemetry telemetry;

    @NonNull
    private final DcMotorImplEx[] motors;

    /**
     * Constructs a new drive subsystems
     * @param opMode The opMode you are running ; To obtain the HardwareMap and Telemetry objects
     */
    public MecanumDriveSubsystem(@NonNull OpMode opMode, @NonNull DriveMode driveMode) {
        shouldHoldPosition = false;
        drive  = 0;
        strafe = 0;
        turn   = 0;

        this.driveMode     = driveMode;
        telemetry          = opMode.telemetry;

        frontLeftMotor  = opMode.hardwareMap.get(DcMotorImplEx.class, FRONT_LEFT_DRIVE_MOTOR_NAME);
        frontRightMotor = opMode.hardwareMap.get(DcMotorImplEx.class, FRONT_RIGHT_DRIVE_MOTOR_NAME);
        backLeftMotor   = opMode.hardwareMap.get(DcMotorImplEx.class, BACK_LEFT_DRIVE_MOTOR_NAME);
        backRightMotor  = opMode.hardwareMap.get(DcMotorImplEx.class, BACK_RIGHT_DRIVE_MOTOR_NAME);

        motors = new DcMotorImplEx[]{
                frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor};

        MotorUtility.setDirections(REVERSE, frontLeftMotor, backLeftMotor);
        MotorUtility.setDirections(FORWARD, frontRightMotor, backRightMotor);

        MotorUtility.setModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER, motors);
        MotorUtility.setZeroPowerBehaviours(BRAKE, motors);
    }

    public void toggleHoldPosition() {
        shouldHoldPosition = !shouldHoldPosition;
    }

    public void setDrivePowers(double drive, double strafe, double turn) {
        this.drive  = drive;
        this.strafe = strafe;
        this.turn   = turn;
    }

    /**
     * Sets the zero power behavior of the robot to float
     */
    public void coast() {
        MotorUtility.setZeroPowerBehaviours(FLOAT, motors);
    }

    public void drive(double drive, double strafe, double turn) {
        double thetaRadians = StrictMath.atan2(drive, strafe);

        double power = StrictMath.hypot(strafe, drive);

        double sin_theta = StrictMath.sin(thetaRadians - Math.PI / 4.0);
        double cos_theta = StrictMath.cos(thetaRadians - Math.PI / 4.0);

        double max = Math.max(Math.abs(cos_theta), Math.abs(sin_theta));

        double frontLeftPower  = power * cos_theta / max + turn;
        double frontRightPower = power * sin_theta / max - turn;
        double backLeftPower   = power * sin_theta / max + turn;
        double backRightPower  = power * cos_theta / max - turn;

        double turnMagnitude = Math.abs(turn);

        if ((power + turnMagnitude) > 1.0) {
            frontLeftPower  /= power + turnMagnitude;
            frontRightPower /= power + turnMagnitude;
            backLeftPower   /= power + turnMagnitude;
            backRightPower  /= power + turnMagnitude;
        }

        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    }

    /**
     * Displays the direction of the drive motors
     */
    public void directionDebug() {
        telemetry.addData("Front Left Direction", frontLeftMotor.getDirection());
        telemetry.addData("Front Right Direction", frontRightMotor.getDirection());
        telemetry.addData("Back Left Direction", backLeftMotor.getDirection());
        telemetry.addData("Back Right Direction", backRightMotor.getDirection());
    }

    /**
     * Displays the current, in amps, of the drive motors
     */
    public void currentDebug() {
        telemetry.addLine("Current Unit: Amps");
        telemetry.addData("Front Left Current", frontLeftMotor.getCurrent(AMPS));
        telemetry.addData("Front Right Current", frontRightMotor.getCurrent(AMPS));
        telemetry.addData("Back Left Current", backLeftMotor.getCurrent(AMPS));
        telemetry.addData("Back Right Current", backRightMotor.getCurrent(AMPS));
    }

    /**
     * Displays the power of the drive motors
     */
    public void powerDebug() {
        telemetry.addData("Front Left Power", frontLeftMotor.getPower());
        telemetry.addData("Front Right Power", frontRightMotor.getPower());
        telemetry.addData("Back Left Power", backLeftMotor.getPower());
        telemetry.addData("Back Right Power", backRightMotor.getPower());
    }

    /**
     * Displays the velocity, in degrees per second, of the drive motors
     */
    public void velocityDebug() {
        telemetry.addLine("Velocity Unit: Degrees/Seconds");
        telemetry.addData("Front Left Velocity", frontLeftMotor.getVelocity(DEGREES));
        telemetry.addData("Front Right Velocity", frontRightMotor.getVelocity(DEGREES));
        telemetry.addData("Back Left Velocity", backLeftMotor.getVelocity(DEGREES));
        telemetry.addData("Back Right Velocity", backRightMotor.getVelocity(DEGREES));
    }

    public void zeroPowerBehaviourDebug() {
        telemetry.addData(
                "Front Left Zero Power Behavior", frontLeftMotor.getZeroPowerBehavior());
        telemetry.addData(
                "Front Right Zero Power Behavior", frontRightMotor.getZeroPowerBehavior());
        telemetry.addData(
                "Back Left Zero Power Behavior", backLeftMotor.getZeroPowerBehavior());
        telemetry.addData(
                "Back Right Zero Power Behavior", backRightMotor.getZeroPowerBehavior());
    }

    /**
     * Displays debug information about the drive base
     */
    public void fullDebug() {
        directionDebug();
        currentDebug();
        powerDebug();
        velocityDebug();
        zeroPowerBehaviourDebug();
    }
}
