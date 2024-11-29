package org.firstinspires.ftc.teamcode.opmodes.teleops.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.mecanumdrive.DriveMode;
import org.firstinspires.ftc.teamcode.subsystems.mecanumdrive.MecanumDrive;

@TeleOp(name = "Test - Drive")
public class DriveTest extends OpMode {
    private MecanumDrive driveBase;

    @Override public void init() {
       driveBase = new MecanumDrive(this, DriveMode.ROBOT_CENTRIC);
    }

    @Override public void loop() {
       driveBase.drive(
               gamepad1.left_stick_y * -1.0,
               gamepad1.left_stick_x,
               gamepad1.right_stick_x
       );
    }
}
