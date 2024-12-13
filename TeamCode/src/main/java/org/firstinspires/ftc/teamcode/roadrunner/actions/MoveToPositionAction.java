package org.firstinspires.ftc.teamcode.roadrunner.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;

public class MoveToPositionAction implements Action {
    private final Arm arm;
    private final ElapsedTime elapsedTime;
    private final double targetVerticalInches;
    private final double targetHorizontalInches;
    private final double timeOutSeconds;

    private boolean isFirstIteration;

    public MoveToPositionAction(
            @NonNull Arm arm,
            double targetVerticalInches,
            double targetHorizontalInches,
            double timeOutSeconds
    ) {
        this.targetVerticalInches   = targetVerticalInches;
        this.targetHorizontalInches = targetHorizontalInches;
        this.arm = arm;
        this.timeOutSeconds = timeOutSeconds;
        elapsedTime = new ElapsedTime();
        isFirstIteration = true;
    }

    @Override public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (isFirstIteration) {
            elapsedTime.reset();
            arm.setTargetPositionInchesRobotCentric(targetVerticalInches, targetHorizontalInches);
            isFirstIteration = false;
        }
        arm.update();

        boolean isFinished = arm.isAtPosition() || elapsedTime.seconds() > timeOutSeconds;
        if (isFinished) arm.stop();

        return isFinished;
    }
}
