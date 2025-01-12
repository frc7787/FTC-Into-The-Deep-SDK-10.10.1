package org.firstinspires.ftc.teamcode.opmodes.auto.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;

public class HomeArmAction implements Action {
    private final Arm arm;

    public HomeArmAction(@NonNull Arm arm) {
        this.arm = arm;
    }

    @Override public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        arm.update();

        return arm.state() != Arm.ArmState.NORMAL;
    }
}
