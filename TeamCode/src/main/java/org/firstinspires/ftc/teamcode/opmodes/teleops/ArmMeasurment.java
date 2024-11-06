package org.firstinspires.ftc.teamcode.opmodes.teleops;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

@TeleOp(name = "Arm Measurement")
public class ArmMeasurment extends OpMode {
    private DcMotorImplEx extensionMotor;

    @Override public void init() {
        extensionMotor = hardwareMap.get(DcMotorImplEx.class, "extensionMotorOne");
        extensionMotor.setZeroPowerBehavior(BRAKE);
        extensionMotor.setMode(STOP_AND_RESET_ENCODER);
        extensionMotor.setMode(RUN_WITHOUT_ENCODER);
    }

    @Override public void loop() {
        telemetry.addData("Extension Motor Position", extensionMotor.getCurrentPosition());
        telemetry.update();
    }
}
