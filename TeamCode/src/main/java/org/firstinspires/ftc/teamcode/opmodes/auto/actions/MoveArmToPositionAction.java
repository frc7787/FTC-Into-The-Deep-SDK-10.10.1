package org.firstinspires.ftc.teamcode.opmodes.auto.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;

public final class MoveArmToPositionAction implements Action {
    private final Arm arm;
    private final double horizontalInches, verticalInches;
    private boolean initialized;

    public MoveArmToPositionAction(
            @NonNull Arm arm,
            double verticalInches,
            double horizontalInches
    ) {
        this.arm = arm;
        this.horizontalInches = horizontalInches;
        this.verticalInches = verticalInches;
        initialized = false;
    }

    @Override public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (!initialized) {
            arm.setTargetPositionInchesRobotCentric(horizontalInches, verticalInches);
            initialized = true;
        }
        arm.update();
        arm.debugGlobal();
        arm.debugPosition();

        boolean isFinished = !arm.isAtPosition();

        if (isFinished) arm.stop();

        return isFinished;
    }
}
