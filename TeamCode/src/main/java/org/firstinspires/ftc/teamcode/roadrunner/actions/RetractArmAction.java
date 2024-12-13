package org.firstinspires.ftc.teamcode.roadrunner.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;

public class RetractArmAction implements Action {
    private final Arm arm;
    private final double extensionPosition;
    private final double timeOutSeconds;
    private final ElapsedTime elapsedTime;
    boolean isFirstIteration = true;

    public RetractArmAction(@NonNull Arm arm, double extensionPosition, double timeOutSeconds) {
        this.arm = arm;
        this.extensionPosition = extensionPosition;
        this.timeOutSeconds = timeOutSeconds;
        elapsedTime = new ElapsedTime();
    }

    @Override public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (isFirstIteration) {
            elapsedTime.reset();
            arm.setExtensionTargetPosition(extensionPosition);
            isFirstIteration = false;
        }
        arm.update();

        boolean isFinished = arm.isAtPosition() || elapsedTime.seconds() > timeOutSeconds;
        if (isFinished) arm.stop();

        return isFinished;
    }
}
