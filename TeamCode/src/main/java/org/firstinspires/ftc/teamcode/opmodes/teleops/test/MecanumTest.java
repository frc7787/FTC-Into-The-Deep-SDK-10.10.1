package org.firstinspires.ftc.teamcode.opmodes.teleops.test;

import static org.firstinspires.ftc.teamcode.subsystems.mecanumdrive.ControlType.SQUARED;
import static org.firstinspires.ftc.teamcode.subsystems.mecanumdrive.MecanumDriveBase.DriveMode.ROBOT_CENTRIC;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.mecanumdrive.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.subsystems.mecanumdrive.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.utility.playstationcontroller.PlayStationController;

@TeleOp(name = "Test - Mecanum")
public final class MecanumTest extends CommandOpMode {

    @Override public void initialize() {
        MecanumDriveBase driveSubsystem = new MecanumDriveBase(this, ROBOT_CENTRIC);

        PlayStationController driverGamepad = new PlayStationController(gamepad1);

        register(driveSubsystem);

        schedule(
                new DriveCommand(
                        driveSubsystem,
                        driverGamepad::getLeftY,
                        driverGamepad::getLeftX,
                        driverGamepad::getRightX,
                        SQUARED
                )
        );
    }
}
