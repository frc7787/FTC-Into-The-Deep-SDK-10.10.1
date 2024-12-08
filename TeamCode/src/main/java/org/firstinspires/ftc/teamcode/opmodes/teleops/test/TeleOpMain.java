package org.firstinspires.ftc.teamcode.opmodes.teleops.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.DriveMode;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;

@TeleOp
public class TeleOpMain extends OpMode {
    private MecanumDrive drive;
    private Arm arm;

    private double INTAKE_OPEN_POSITION   = 0.60;
    private double INTAKE_CLOSED_POSITION = 0.00;

    @Override public void init() {
        drive = new MecanumDrive.Builder(hardwareMap)
                .setDriveMode(DriveMode.ROBOT_CENTRIC)
                .build();
        arm = new Arm(this);
    }

    @Override public void loop() {

        if (gamepad2.dpad_down) {
            arm.setTargetPosition(10.0, -3.0);
            arm.setIntakePosition(INTAKE_OPEN_POSITION);
        } else if (gamepad2.dpad_up) {
            arm.setTargetPosition(10.0, 0.0);
            arm.setIntakePosition(INTAKE_CLOSED_POSITION);
        }

        arm.debugPosition();
        arm.update();
    }
}
