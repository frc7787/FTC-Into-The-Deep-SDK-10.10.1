package org.firstinspires.ftc.teamcode.vision.color;

import androidx.annotation.NonNull;

import org.opencv.core.Rect;

public final class SampleDetection {
    @NonNull
    public final Rect rect;
    @NonNull
    public final SampleColor color;

    public SampleDetection(@NonNull Rect rect, @NonNull SampleColor color) {
        this.rect  = rect;
        this.color = color;
    }
}