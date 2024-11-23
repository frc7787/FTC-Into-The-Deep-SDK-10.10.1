package org.firstinspires.ftc.teamcode.opmodes.teleops.utility;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;

@TeleOp(name = "Measurement - Arm", group = "Utility")
public class ArmMeasurment extends OpMode {
    private Arm arm;

    @Override public void init() {
        arm = new Arm(this);
    }

    @Override public void loop() {
        arm.update();
        arm.debugSetArmPower(gamepad2.left_stick_y, gamepad2.right_stick_y);

        arm.debugPositionInformation();
    }
}
