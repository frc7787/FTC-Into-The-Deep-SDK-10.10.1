package org.firstinspires.ftc.teamcode.opmodes.teleops.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;

@TeleOp(name = "Measurement - Arm", group = "Measurement")
public class ArmMeasurement extends OpMode {
    private Arm arm;

    @Override public void init() {
       arm = new Arm(this);
    }

    @Override public void loop() {
        arm.debugSetArmPower(
                0,
                gamepad1.left_stick_y * -1.0
        );

        arm.debugRotation();
    }
}
