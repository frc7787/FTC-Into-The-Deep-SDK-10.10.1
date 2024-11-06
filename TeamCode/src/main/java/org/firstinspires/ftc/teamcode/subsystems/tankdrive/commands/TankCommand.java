package org.firstinspires.ftc.teamcode.subsystems.tankdrive.commands;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.tankdrive.TankDriveSubsystem;

import java.util.function.DoubleSupplier;

public final class TankCommand extends CommandBase {
    private final TankDriveSubsystem tankDriveSubsystem;
    private final DoubleSupplier leftPowerSupplier, rightPowerSupplier;
    private final boolean squareInputs;

    public TankCommand(
            @NonNull TankDriveSubsystem tankDriveSubsystem,
            @NonNull DoubleSupplier leftPowerSupplier,
            @NonNull DoubleSupplier rightPowerSupplier,
            boolean squareInputs
    ) {
        this.tankDriveSubsystem = tankDriveSubsystem;
        this.leftPowerSupplier  = leftPowerSupplier;
        this.rightPowerSupplier = rightPowerSupplier;
        this.squareInputs       = squareInputs;

        addRequirements(tankDriveSubsystem);
    }

    @Override public void execute() {
        double rawLeftPower  = leftPowerSupplier.getAsDouble();
        double rawRightPower = rightPowerSupplier.getAsDouble();

        double leftPower, rightPower;

        if (squareInputs) {
            leftPower  = Math.abs(rawLeftPower) * rawLeftPower;
            rightPower = Math.abs(rawRightPower) * rawRightPower;
        } else {
            leftPower  = rawLeftPower;
            rightPower = rawRightPower;
        }

        tankDriveSubsystem.tank(leftPower, rightPower);
    }
}
