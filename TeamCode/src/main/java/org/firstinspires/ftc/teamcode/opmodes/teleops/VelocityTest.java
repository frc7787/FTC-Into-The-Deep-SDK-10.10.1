package org.firstinspires.ftc.teamcode.opmodes.teleops;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

import org.firstinspires.ftc.teamcode.utility.MotorUtility;

@TeleOp(name = "Test - Velocity")
public class VelocityTest extends OpMode {
    private DcMotorImplEx extensionMotorOne, extensionMotorTwo;

    private double maxTicksPerSecond;

    @Override public void init() {
        extensionMotorOne = hardwareMap.get(DcMotorImplEx.class, "extensionMotorOne");
        extensionMotorTwo = hardwareMap.get(DcMotorImplEx.class, "extensionMotorTwo");
        maxTicksPerSecond = 0.0;

        MotorUtility.setZeroPowerBehaviours(BRAKE, extensionMotorOne, extensionMotorTwo);
        MotorUtility.setModes(RUN_WITHOUT_ENCODER, extensionMotorOne, extensionMotorTwo);
    }

    @Override public void loop() {
        double leftStickY = gamepad1.left_stick_y * -1.0;

        extensionMotorOne.setPower(leftStickY);
        extensionMotorTwo.setPower(leftStickY);

        double ticksPerSecond = extensionMotorOne.getVelocity();

        if (Math.abs(ticksPerSecond) > Math.abs(maxTicksPerSecond)) {
            maxTicksPerSecond = ticksPerSecond;
        }

        telemetry.addData("Current Position", extensionMotorOne.getCurrentPosition());
        telemetry.addData("Current Ticks Per Second", ticksPerSecond);
        telemetry.addData("Max Ticks Per Second", maxTicksPerSecond);
        telemetry.update();
    }
}
