package org.firstinspires.ftc.teamcode.subsystems.mecanumdrive;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.*;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.*;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.*;
import static org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.*;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.*;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.*;
import org.firstinspires.ftc.teamcode.utility.MotorUtility;

@LoadStaticFields
public final class MecanumDrive extends SubsystemBase {
    private static final String FRONT_LEFT_DRIVE_MOTOR_NAME  = "frontLeftDriveMotor";
    private static final String FRONT_RIGHT_DRIVE_MOTOR_NAME = "frontRightDriveMotor";
    private static final String BACK_LEFT_DRIVE_MOTOR_NAME   = "backLeftDriveMotor";
    private static final String BACK_RIGHT_DRIVE_MOTOR_NAME  = "backRightDriveMotor";
    private static final String IMU_NAME                     = "imu";

    // ---------- Hardware ---------- //

    @NonNull private final DcMotorImplEx frontLeftMotor,
                                         frontRightMotor,
                                         backLeftMotor,
                                         backRightMotor;

    @NonNull private final GoBildaPinpointDriver odometry;

    @NonNull private final DcMotorImplEx[] motors;

    // ---------- Configuration ----------- //

    @NonNull private DriveMode driveMode;

    // ---------- Debug ----------- //

    @NonNull private final Telemetry telemetry;

    /**
     * Constructs a new drive subsystems
     * @param opMode The opMode you are running ; To obtain the HardwareMap and Telemetry objects
     */
    public MecanumDrive(@NonNull OpMode opMode, @NonNull DriveMode driveMode) {
        this.driveMode = driveMode;
        telemetry      = opMode.telemetry;

        frontLeftMotor  = opMode.hardwareMap.get(DcMotorImplEx.class, FRONT_LEFT_DRIVE_MOTOR_NAME);
        frontRightMotor = opMode.hardwareMap.get(DcMotorImplEx.class, FRONT_RIGHT_DRIVE_MOTOR_NAME);
        backLeftMotor   = opMode.hardwareMap.get(DcMotorImplEx.class, BACK_LEFT_DRIVE_MOTOR_NAME);
        backRightMotor  = opMode.hardwareMap.get(DcMotorImplEx.class, BACK_RIGHT_DRIVE_MOTOR_NAME);

        motors = new DcMotorImplEx[]{
                frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor};

        odometry = opMode.hardwareMap.get(GoBildaPinpointDriver.class, "odometry");
        odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odometry.recalibrateIMU();

        MotorUtility.setDirections(REVERSE, frontLeftMotor, backLeftMotor);
        MotorUtility.setDirections(FORWARD, frontRightMotor, backRightMotor);

        MotorUtility.setModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER, motors);
        MotorUtility.setZeroPowerBehaviours(BRAKE, motors);
    }

    public void drive(double drive, double strafe, double turn) {
        double thetaRadians = StrictMath.atan2(drive, strafe);

        if (driveMode == DriveMode.FIELD_CENTRIC) {
            thetaRadians -= odometry.getHeading();
        }

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

    public void resetYaw() { odometry.resetPosAndIMU(); }

    public void directionDebug() {
        telemetry.addLine("----- Direction -----");
        telemetry.addData("Front Left", frontLeftMotor.getDirection());
        telemetry.addData("Front Right", frontRightMotor.getDirection());
        telemetry.addData("Back Left", backLeftMotor.getDirection());
        telemetry.addData("Back Right", backRightMotor.getDirection());
    }

    public void currentDebug() {
        telemetry.addLine("----- Current (Amps) -----");
        telemetry.addData("Front Left", frontLeftMotor.getCurrent(AMPS));
        telemetry.addData("Front Right", frontRightMotor.getCurrent(AMPS));
        telemetry.addData("Back Left", backLeftMotor.getCurrent(AMPS));
        telemetry.addData("Back Right", backRightMotor.getCurrent(AMPS));
    }

    public void powerDebug() {
        telemetry.addLine("----- Power -----");
        telemetry.addData("Front Left", frontLeftMotor.getPower());
        telemetry.addData("Front Right", frontRightMotor.getPower());
        telemetry.addData("Back Left", backLeftMotor.getPower());
        telemetry.addData("Back Right", backRightMotor.getPower());
    }

    public void velocityDebug() {
        telemetry.addLine("----- Velocity (Degrees/Sec) ----- ");
        telemetry.addData("Front Left", frontLeftMotor.getVelocity(DEGREES));
        telemetry.addData("Front Right", frontRightMotor.getVelocity(DEGREES));
        telemetry.addData("Back Left", backLeftMotor.getVelocity(DEGREES));
        telemetry.addData("Back Right", backRightMotor.getVelocity(DEGREES));
    }

    public void zeroPowerBehaviourDebug() {
        telemetry.addLine("----- Zero Power Behaviour -----");
        telemetry.addData("Front Left", frontLeftMotor.getZeroPowerBehavior());
        telemetry.addData("Front Right", frontRightMotor.getZeroPowerBehavior());
        telemetry.addData("Back Left", backLeftMotor.getZeroPowerBehavior());
        telemetry.addData("Back Right", backRightMotor.getZeroPowerBehavior());
    }

    public void debugIMU() {
        telemetry.addLine("----- IMU -----");
        telemetry.addData("Heading (Radians)", odometry.getHeading());
        telemetry.addData("Velocity(Radians/Sec)", odometry.getHeadingVelocity());
    }

    public void fullDebug() {
        directionDebug();
        currentDebug();
        powerDebug();
        velocityDebug();
        zeroPowerBehaviourDebug();
    }
}
