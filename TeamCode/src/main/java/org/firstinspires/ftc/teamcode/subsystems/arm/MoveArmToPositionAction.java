package org.firstinspires.ftc.teamcode.subsystems.arm;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class MoveArmToPositionAction implements Action {
    private final Arm arm;
    private final double inches, degrees;

    private boolean initialized;

    public MoveArmToPositionAction(@NonNull Arm arm, double inches, double degrees) {
        initialized = false;

        this.arm     = arm;
        this.inches  = inches;
        this.degrees = degrees;
    }

    @Override public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (!initialized) {
            arm.setTargetPosition(inches, degrees);
            initialized = true;
        }

        arm.update();

        return arm.state() != Arm.ArmState.NORMAL;
    }
}
