package org.firstinspires.ftc.teamcode.utility;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public class Debugger {
    protected final ArrayList<String> output;

    @Nullable
    public final Telemetry telemetry;

    public Debugger(@Nullable Telemetry telemetry) {
        this.output    = new ArrayList<>();
        this.telemetry = telemetry;
    }

    public void addMessage(@NonNull String message) {
        output.add(message);
    }

    public void displayAll() {
        if (telemetry == null) return;

        telemetry.addData("Number Of Issues", output.size());

        for (String message: output) { telemetry.addLine("\n" + message); }

        telemetry.update();
    }
}


