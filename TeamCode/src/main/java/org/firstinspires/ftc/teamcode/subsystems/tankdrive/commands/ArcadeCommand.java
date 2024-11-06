package org.firstinspires.ftc.teamcode.subsystems.tankdrive.commands;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.tankdrive.TankDriveSubsystem;

import java.util.function.DoubleSupplier;

public final class ArcadeCommand extends CommandBase {
    private final TankDriveSubsystem tankDriveSubsystem;
    private final DoubleSupplier driveSupplier, turnSupplier;
    private final boolean squareInputs;

    public ArcadeCommand(
            @NonNull TankDriveSubsystem tankDriveSubsystem,
            @NonNull DoubleSupplier driveSupplier,
            @NonNull DoubleSupplier turnSupplier,
            boolean squareInputs
    ) {
        this.tankDriveSubsystem = tankDriveSubsystem;
        this.driveSupplier      = driveSupplier;
        this.turnSupplier       = turnSupplier;
        this.squareInputs       = squareInputs;

        addRequirements(tankDriveSubsystem);
    }

    @Override public void execute() {
        double drivePowerRaw = driveSupplier.getAsDouble();
        double turnPowerRaw  = turnSupplier.getAsDouble();

        double drivePower = drivePowerRaw;
        double turnPower  = turnPowerRaw;

        if (squareInputs) {
            drivePower = Math.abs(drivePowerRaw) * drivePowerRaw;
            turnPower  = Math.abs(turnPowerRaw) * turnPowerRaw;
        }

        tankDriveSubsystem.arcade(drivePower, turnPower);
    }
}
