package org.firstinspires.ftc.teamcode.roadrunner.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;

public class HomeAction implements Action {
    private final Arm arm;
    private final ElapsedTime timer;
    private boolean isInitialized;

    public HomeAction(@NonNull Arm arm) {
        this.arm = arm;
        timer = new ElapsedTime();
        isInitialized = false;
    }

    @Override public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (!isInitialized) {
            timer.reset();
            isInitialized = true;
        }
        arm.update();
        return arm.state() != Arm.ArmState.HOMING || timer.seconds() > 5;
    }
}
