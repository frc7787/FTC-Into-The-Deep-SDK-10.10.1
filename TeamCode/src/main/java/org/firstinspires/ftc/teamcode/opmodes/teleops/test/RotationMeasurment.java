package org.firstinspires.ftc.teamcode.opmodes.teleops.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class RotationMeasurment extends OpMode {
    private DcMotor rotationMotor;

    @Override public void init() {
        rotationMotor = hardwareMap.get(DcMotor.class, "rotationMotor");
    }

    @Override public void loop() {
        telemetry.addData("Rotation Position", rotationMotor.getCurrentPosition());
        rotationMotor.setPower(gamepad1.left_stick_y);
    }
}
