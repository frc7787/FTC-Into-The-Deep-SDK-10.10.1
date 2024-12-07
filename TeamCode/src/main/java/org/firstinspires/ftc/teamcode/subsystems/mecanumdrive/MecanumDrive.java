package org.firstinspires.ftc.teamcode.subsystems.mecanumdrive;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.*;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.*;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.*;
import static org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.*;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.utility.MotorUtility;

public final class MecanumDrive {
    // ---------------------------------------------------------------------------------------------
    // Hardware
    // ---------------------------------------------------------------------------------------------

    @NonNull private final DcMotorImplEx frontLeftDriveMotor,
                                         frontRightDriveMotor,
                                         backLeftDriveMotor,
                                         backRightDriveMotor;


    @NonNull private final DcMotorImplEx[] motors;

    // ---------------------------------------------------------------------------------------------
    // Debug
    // ---------------------------------------------------------------------------------------------

    @NonNull private final Telemetry telemetry;

    // ---------------------------------------------------------------------------------------------
    // Constructor
    // ---------------------------------------------------------------------------------------------

    /**
     * Constructs a new drive subsystems
     * @param opMode The opMode you are running ; To obtain the HardwareMap and Telemetry objects
     */
    public MecanumDrive(@NonNull OpMode opMode) {
        telemetry = opMode.telemetry;

        frontLeftDriveMotor  = opMode.hardwareMap.get(DcMotorImplEx.class, "frontLeftDriveMotor");
        frontRightDriveMotor = opMode.hardwareMap.get(DcMotorImplEx.class, "frontRightDriveMotor");
        backLeftDriveMotor   = opMode.hardwareMap.get(DcMotorImplEx.class, "backLeftDriveMotor");
        backRightDriveMotor  = opMode.hardwareMap.get(DcMotorImplEx.class, "backRightDriveMotor");

        motors = new DcMotorImplEx[]{
                frontLeftDriveMotor, frontRightDriveMotor, backLeftDriveMotor, backRightDriveMotor};

        MotorUtility.setDirections(REVERSE, frontLeftDriveMotor, backLeftDriveMotor);
        MotorUtility.reset(motors);
        MotorUtility.setZeroPowerBehaviours(BRAKE, motors);
    }

    // ---------------------------------------------------------------------------------------------
    // Drive Methods
    // ---------------------------------------------------------------------------------------------

    public void drive(double drive, double strafe, double turn, DriveMode mode) {

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

        frontLeftDriveMotor.setPower(frontLeftPower);
        frontRightDriveMotor.setPower(frontRightPower);
        backLeftDriveMotor.setPower(backLeftPower);
        backRightDriveMotor.setPower(backRightPower);
    }

    // ---------------------------------------------------------------------------------------------
    // Getters
    // ---------------------------------------------------------------------------------------------

    public double currentAmps(CurrentUnit unit) {
        return frontLeftDriveMotor.getCurrent(unit) + frontRightDriveMotor.getCurrent(unit)
               + backLeftDriveMotor.getCurrent(unit) + backRightDriveMotor.getCurrent(unit);
    }

    // ---------------------------------------------------------------------------------------------
    // Debug
    // ---------------------------------------------------------------------------------------------

    public void directionDebug() {
        telemetry.addLine("----- Direction -----");
        telemetry.addData("Front Left", frontLeftDriveMotor.getDirection());
        telemetry.addData("Front Right", frontRightDriveMotor.getDirection());
        telemetry.addData("Back Left", backLeftDriveMotor.getDirection());
        telemetry.addData("Back Right", backRightDriveMotor.getDirection());
    }

    public void currentDebug() {
        telemetry.addLine("----- Current (Amps) -----");
        telemetry.addData("Front Left", frontLeftDriveMotor.getCurrent(AMPS));
        telemetry.addData("Front Right", frontRightDriveMotor.getCurrent(AMPS));
        telemetry.addData("Back Left", backLeftDriveMotor.getCurrent(AMPS));
        telemetry.addData("Back Right", backRightDriveMotor.getCurrent(AMPS));
    }

    public void powerDebug() {
        telemetry.addLine("----- Power -----");
        telemetry.addData("Front Left", frontLeftDriveMotor.getPower());
        telemetry.addData("Front Right", frontRightDriveMotor.getPower());
        telemetry.addData("Back Left", backLeftDriveMotor.getPower());
        telemetry.addData("Back Right", backRightDriveMotor.getPower());
    }

    public void velocityDebug() {
        telemetry.addLine("----- Velocity (Degrees/Sec) ----- ");
        telemetry.addData("Front Left", frontLeftDriveMotor.getVelocity(DEGREES));
        telemetry.addData("Front Right", frontRightDriveMotor.getVelocity(DEGREES));
        telemetry.addData("Back Left", backLeftDriveMotor.getVelocity(DEGREES));
        telemetry.addData("Back Right", backRightDriveMotor.getVelocity(DEGREES));
    }

    public void zeroPowerBehaviourDebug() {
        telemetry.addLine("----- Zero Power Behaviour -----");
        telemetry.addData("Front Left", frontLeftDriveMotor.getZeroPowerBehavior());
        telemetry.addData("Front Right", frontRightDriveMotor.getZeroPowerBehavior());
        telemetry.addData("Back Left", backLeftDriveMotor.getZeroPowerBehavior());
        telemetry.addData("Back Right", backRightDriveMotor.getZeroPowerBehavior());
    }

    public void fullDebug() {
        directionDebug();
        currentDebug();
        powerDebug();
        velocityDebug();
        zeroPowerBehaviourDebug();
    }
}
