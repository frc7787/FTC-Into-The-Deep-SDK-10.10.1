package org.firstinspires.ftc.teamcode.subsystems.tankdrive;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection.UP;
import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.utility.MotorUtility;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.util.Range;

public final class TankDriveSubsystem extends SubsystemBase {
    private final DcMotorImplEx leftMotor, rightMotor;
    private final IMU imu;

    public TankDriveSubsystem(@NonNull OpMode opMode) {
        HardwareMap hardwareMap = opMode.hardwareMap;

        leftMotor  = hardwareMap.get(DcMotorImplEx.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotorImplEx.class, "rightMotor");

        leftMotor.setDirection(Direction.REVERSE);

        MotorUtility.setZeroPowerBehaviours(BRAKE, leftMotor, rightMotor);

        imu = hardwareMap.get(IMU.class, "imu");

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(UP, FORWARD)));
        imu.resetYaw();
    }

    public void tank(double leftPower, double rightPower) {
        if (Math.abs(leftPower) < 0.02)  leftPower = 0.0;
        if (Math.abs(rightPower) < 0.02) rightPower = 0.0;

        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
    }

    public void arcade(double drivePower, double turnPower) {
        if (Math.abs(drivePower) < 0.02) drivePower = 0.0;
        if (Math.abs(turnPower) < 0.02)  turnPower = 0.0;

        double leftPower  = Range.clip(drivePower + turnPower, -1.0, 1.0);
        double rightPower = Range.clip(drivePower - turnPower, -1.0, 1.0);

        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
    }

    public void curvature(
            double forwardPower,
            double reversePower,
            double rotationPower,
            boolean turnOverride
    ) {
        if (Math.abs(forwardPower)  < 0.02)  forwardPower  = 0.0;
        if (Math.abs(rotationPower) < 0.02)  rotationPower = 0.0;
        if (Math.abs(reversePower)  < 0.02)  reversePower  = 0.0;

        double drivePower = forwardPower + reversePower;

        double leftSpeed, rightSpeed;

        if (turnOverride) {
            leftSpeed  = drivePower + rotationPower;
            rightSpeed = drivePower - rotationPower;
        } else {
            leftSpeed  = drivePower + Math.abs(drivePower) * rotationPower;
            rightSpeed = drivePower -  Math.abs(drivePower) * rotationPower;
        }

        double maxMagnitude = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));

        if (maxMagnitude > 1.0) {
            leftSpeed  /= maxMagnitude;
            rightSpeed /= maxMagnitude;
        }

        leftMotor.setPower(leftSpeed);
        rightMotor.setPower(rightSpeed);
    }
}
