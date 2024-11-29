package org.firstinspires.ftc.teamcode.opmodes.teleops.test;

import static org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

@TeleOp(name = "Test - Rotation Motor", group = "Test")
public class RotationMotorTest extends OpMode {
    private DcMotorImplEx rotationMotor;

    @Override public void init() {
        rotationMotor = hardwareMap.get(DcMotorImplEx.class, "rotationMotor");
    }

    @Override public void loop() {
        rotationMotor.setPower(gamepad1.left_stick_y);
        telemetry.addData("Velocity", rotationMotor.getVelocity());
        telemetry.addData("Rotation Current (Amps)", rotationMotor.getCurrent(AMPS));
    }
}
