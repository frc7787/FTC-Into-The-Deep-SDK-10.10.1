package org.firstinspires.ftc.teamcode.subsystems.mecanumdrive.commands;

import static org.firstinspires.ftc.teamcode.subsystems.mecanumdrive.ControlType.SQUARED;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.mecanumdrive.ControlType;
import org.firstinspires.ftc.teamcode.subsystems.mecanumdrive.MecanumDriveSubsystem;
import java.util.function.DoubleSupplier;

public final class DriveCommand extends CommandBase {
        @NonNull
        private final MecanumDriveSubsystem mecanumDriveSubsystem;

        @NonNull
        private final DoubleSupplier driveSupplier, strafeSupplier, turnSupplier;

        @NonNull
        private final ControlType controlType;

        public DriveCommand(
                @NonNull MecanumDriveSubsystem mecanumDriveSubsystem,
                @NonNull DoubleSupplier driveSupplier,
                @NonNull DoubleSupplier strafeSupplier,
                @NonNull DoubleSupplier turnSupplier,
                @NonNull ControlType controlType
        ) {
            this.mecanumDriveSubsystem = mecanumDriveSubsystem;
            this.driveSupplier         = driveSupplier;
            this.strafeSupplier        = strafeSupplier;
            this.turnSupplier          = turnSupplier;
            this.controlType           = controlType;

            addRequirements(mecanumDriveSubsystem);
        }

        @Override public void execute() {
            double driveValue  = driveSupplier.getAsDouble();
            double turnValue   = turnSupplier.getAsDouble();
            double strafeValue = strafeSupplier.getAsDouble();

            if (Math.abs(driveValue) <= 0.05) driveValue = 0.0;
            if (Math.abs(turnValue) <= 0.05) turnValue = 0.0;
            if (Math.abs(strafeValue) <= 0.05) strafeValue = 0.0;

            mecanumDriveSubsystem.setDrivePowers(driveValue, strafeValue, turnValue);
        }
}
