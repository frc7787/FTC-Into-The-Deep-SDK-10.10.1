package org.firstinspires.ftc.teamcode.opmodes.test;

import static org.firstinspires.ftc.teamcode.subsystems.mecanumdrive.ControlType.SQUARED;
import static org.firstinspires.ftc.teamcode.subsystems.mecanumdrive.MecanumDriveSubsystem.DriveMode.ROBOT_CENTRIC;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.mecanumdrive.ControlType;
import org.firstinspires.ftc.teamcode.subsystems.mecanumdrive.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.mecanumdrive.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.utility.playstationcontroller.PlayStationController;

@TeleOp(name = "Test - Mecanum")
@Disabled
public final class MecanumTest extends CommandOpMode {

    @Override public void initialize() {
        MecanumDriveSubsystem mecanumDriveSubsystem = new MecanumDriveSubsystem(this, ROBOT_CENTRIC);
        PlayStationController driverController      = new PlayStationController(gamepad1);

        register(mecanumDriveSubsystem);

        schedule(
                new DriveCommand(
                        mecanumDriveSubsystem,
                        driverController::getLeftY,
                        driverController::getLeftX,
                        driverController::getRightX,
                        SQUARED
                )
        );
    }
}
