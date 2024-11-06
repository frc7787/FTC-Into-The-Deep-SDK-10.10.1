package org.firstinspires.ftc.teamcode.subsystems.arm.commands;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem;

public final class SetArmTargetPositionCommand extends CommandBase {
    private final ArmSubsystem armSubsystem;
    private final int rotationTargetPosition, extensionTargetPosition;

    public SetArmTargetPositionCommand(
            @NonNull ArmSubsystem armSubsystem,
            int rotationTargetPosition,
            int extensionTargetPosition
    ) {
        this.armSubsystem = armSubsystem;
        this.rotationTargetPosition  = rotationTargetPosition;
        this.extensionTargetPosition = extensionTargetPosition;
    }

    @Override public void execute() {
        armSubsystem.rotationTargetPosition  = rotationTargetPosition;
        armSubsystem.extensionTargetPosition = extensionTargetPosition;
    }

    @Override public boolean isFinished() {
        return true;
    }
}
