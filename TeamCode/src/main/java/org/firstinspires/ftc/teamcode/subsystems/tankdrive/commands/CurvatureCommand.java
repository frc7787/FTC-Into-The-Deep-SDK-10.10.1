package org.firstinspires.ftc.teamcode.subsystems.tankdrive.commands;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.tankdrive.TankDriveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public final class CurvatureCommand extends CommandBase {
    private final TankDriveSubsystem tankDriveSubsystem;
    private final DoubleSupplier forwardSupplier, reverseSupplier, rotationSupplier;
    private final BooleanSupplier turnOverrideSupplier;

    public CurvatureCommand(
            @NonNull TankDriveSubsystem tankDriveSubsystem,
            @NonNull DoubleSupplier forwardSupplier,
            @NonNull DoubleSupplier reverseSupplier,
            @NonNull DoubleSupplier rotationSupplier,
            @NonNull BooleanSupplier turnOverrideSupplier
    ) {
        this.tankDriveSubsystem   = tankDriveSubsystem;
        this.forwardSupplier      = forwardSupplier;
        this.reverseSupplier      = reverseSupplier;
        this.rotationSupplier     = rotationSupplier;
        this.turnOverrideSupplier = turnOverrideSupplier;

        addRequirements(tankDriveSubsystem);
    }

    @Override public void execute() {
        tankDriveSubsystem.curvature(
                forwardSupplier.getAsDouble(),
                reverseSupplier.getAsDouble(),
                rotationSupplier.getAsDouble(),
                turnOverrideSupplier.getAsBoolean()
        );
    }
}
