package org.firstinspires.ftc.teamcode.opmodes.teleops.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;


@TeleOp
public class LinearRegressionTesting extends OpMode {
    private Arm arm;

    @Override public void init() {
       arm = new Arm(this);
    }

    @Override public void loop() {
       if (gamepad1.options) arm.update();
       arm.debugPosition();
    }
}
