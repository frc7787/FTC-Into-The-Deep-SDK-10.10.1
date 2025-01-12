package org.firstinspires.ftc.teamcode.opmodes.auto.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;

public class OpenGripperAction implements Action {
    private final Arm arm;
    private final double position;
    private final ElapsedTime timer;

    private boolean initialized;

    public OpenGripperAction(@NonNull Arm arm, double position) {
        this.arm = arm;
        this.position = position;
        initialized = false;
        timer = new ElapsedTime();
    }

    @Override public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (!initialized) {
            arm.setIntakePosition(position);
            timer.reset();
            initialized = true;
        }

        return timer.milliseconds() < 100;
    }
}
