package org.firstinspires.ftc.teamcode.opmodes.teleops.debug;

import static org.firstinspires.ftc.teamcode.subsystems.mecanumdrive.ControlType.SQUARED;
import static org.firstinspires.ftc.teamcode.subsystems.mecanumdrive.MecanumDriveSubsystem.DriveMode.ROBOT_CENTRIC;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.mecanumdrive.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.mecanumdrive.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.utility.playstationcontroller.PlayStationController;

@TeleOp(name = "Debug - Pinpoint", group = "Debug")
public class PinpointDebug extends CommandOpMode {

    @Override public void initialize() {
        MecanumDriveSubsystem driveSubsystem = new MecanumDriveSubsystem(this, ROBOT_CENTRIC);

        register(driveSubsystem);

        PlayStationController driverGamepad = new PlayStationController(gamepad1);

        schedule(
                new DriveCommand(
                        driveSubsystem,
                        driverGamepad::getLeftY,
                        driverGamepad::getLeftX,
                        driverGamepad::getRightX,
                        SQUARED
                ),
                new InstantCommand(driveSubsystem::pinpointDebug),
                new InstantCommand(telemetry::update)
        );
    }
}
