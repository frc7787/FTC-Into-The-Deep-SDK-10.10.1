package org.firstinspires.ftc.teamcode.utility.playstationcontroller;

import com.qualcomm.robotcore.util.Range;

public final class RGB {
    public final int RED, GREEN, BLUE;

    public RGB(int red, int green, int blue) {
        this.RED   = Range.clip(red, 0, 255);
        this.GREEN = Range.clip(green, 0, 255);
        this.BLUE  = Range.clip(blue, 0, 255);
    }

}
