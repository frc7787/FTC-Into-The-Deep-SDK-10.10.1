package org.firstinspires.ftc.teamcode.opmodes.test;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.tankdrive.TankDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.tankdrive.commands.CurvatureCommand;
import org.firstinspires.ftc.teamcode.utility.playstationcontroller.PlayStationController;

@TeleOp(name = "Tank Drive", group = "Drive Only")
@Disabled
public final class TankTest extends CommandOpMode {
    private TankDriveSubsystem tankDriveSubsystem;

    @Override public void initialize() {
        tankDriveSubsystem = new TankDriveSubsystem(this);

        PlayStationController driverController = new PlayStationController(gamepad1);

        tankDriveSubsystem.setDefaultCommand(
                new CurvatureCommand(
                        tankDriveSubsystem,
                        driverController::leftTrigger,
                        () -> driverController.leftTrigger() * -1.0,
                        driverController::getLeftX,
                        driverController::leftBumper
                )
        );
    }
}
