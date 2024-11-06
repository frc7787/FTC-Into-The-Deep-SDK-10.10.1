package org.firstinspires.ftc.teamcode.utility.playstationcontroller;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Wrapper class around GamepadKeys.Button enum
 * to accurately represent PS4/PS5 bindings
 */
public final class PlayStationController extends GamepadEx {
    public static Button LEFT_BUMPER  = Button.LEFT_BUMPER;
    public static Button RIGHT_BUMPER = Button.RIGHT_BUMPER;
    public static Button DPAD_UP      = Button.DPAD_UP;
    public static Button DPAD_DOWN    = Button.DPAD_DOWN;
    public static Button DPAD_LEFT    = Button.DPAD_LEFT;
    public static Button DPAD_RIGHT   = Button.DPAD_RIGHT;
    public static Button TRIANGLE     = Button.Y;
    public static Button SQUARE       = Button.X;
    public static Button CROSS        = Button.A;
    public static Button CIRCLE       = Button.B;
    public static Button SHARE        = Button.BACK;
    public static Button OPTIONS      = Button.START;

    public final Gamepad gamepad;

    public PlayStationController(@NonNull Gamepad gamepad) {
        super(gamepad);

        this.gamepad = gamepad;
    }

    public double leftTrigger() {
        return gamepad.left_trigger;
    }

    public double rightTrigger() {
        return gamepad.right_trigger;
    }

    public boolean leftBumper() {
        return gamepad.left_bumper;
    }

    public boolean rightBumper() {
        return gamepad.right_bumper;
    }

    public boolean cross() {
        return gamepad.cross;
    }

    public boolean square() {
        return gamepad.square;
    }

    public boolean triangle() {
        return gamepad.triangle;
    }

    public boolean circle() {
        return gamepad.circle;
    }

    public boolean dpadUp() {
        return gamepad.dpad_up;
    }

    public boolean dpadDown() {
        return gamepad.dpad_down;
    }

    public boolean dpadLeft() {
        return gamepad.dpad_left;
    }

    public boolean dpadRight() {
        return gamepad.dpad_right;
    }
}
