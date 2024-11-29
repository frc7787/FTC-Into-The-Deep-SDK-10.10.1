package org.firstinspires.ftc.teamcode.opmodes.teleops.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.mecanumdrive.DriveMode;
import org.firstinspires.ftc.teamcode.subsystems.mecanumdrive.MecanumDrive;

@TeleOp(name = "Test - Mecanum")
public final class MecanumTest extends OpMode {
    private MecanumDrive driveBase;

    @Override public void init() {
       driveBase = new MecanumDrive(this, DriveMode.ROBOT_CENTRIC);
    }

    @Override public void loop() {
        double drive  = gamepad1.left_stick_y * -1.0;
        double strafe = gamepad1.left_stick_x;
        double turn   = gamepad1.right_stick_x;

        if (Math.abs(drive) < 0.05)  drive  = 0.0;
        if (Math.abs(strafe) < 0.05) strafe = 0.0;
        if (Math.abs(turn) < 0.05)   turn   = 0.0;

        drive  *= Math.abs(drive);
        strafe *= Math.abs(strafe);
        turn   *= Math.abs(turn);

        driveBase.drive(drive, strafe, turn);
    }
}
