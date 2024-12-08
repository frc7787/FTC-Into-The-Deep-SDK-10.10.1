package org.firstinspires.ftc.teamcode.opmodes.teleops.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.DriveMode;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@TeleOp
public class MecanumTest extends OpMode {
    private MecanumDrive mecanumDrive;

    @Override public void init() {
        mecanumDrive = new MecanumDrive.Builder(hardwareMap)
                .setDriveMode(DriveMode.ROBOT_CENTRIC)
                .build();
    }

    @Override public void loop() {
       mecanumDrive.drive(
               gamepad1.left_stick_y * -1.0,
               gamepad1.left_stick_x,
               gamepad1.right_stick_x
       );
    }
}
