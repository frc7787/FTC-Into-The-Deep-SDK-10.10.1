package org.firstinspires.ftc.teamcode.utility.playstationcontroller.commands;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.utility.playstationcontroller.RGB;

public final class ChangeLEDColorCommand extends CommandBase {
	private final Gamepad gamepad;
	private final int       red, green, blue;
	private final int       durationMS;

	/**
	 * Changes the color of the PS4/PS5 controller LED for the specified duration
	 * @param gamepad    The PS4/PS5 controller to change the color of.
	 * @param color      The color to set the LED to
	 * @param durationMS The duration of the color change
	 */
	public ChangeLEDColorCommand(
			@NonNull GamepadEx gamepad,
			@NonNull RGB color,
			int durationMS
	) {
		this.gamepad    = gamepad.gamepad;
		this.red        = color.RED;
		this.green      = color.GREEN;
		this.blue       = color.BLUE;
		this.durationMS = durationMS;
	}

	/**
	 * Changes the color of the PS4/PS5 controller LED.
	 * @param gamepad The controller to change the color of
	 * @param color The color to set the gamepad to
	 */
	public ChangeLEDColorCommand(
			@NonNull GamepadEx gamepad,
			@NonNull RGB color
	) {
		this.gamepad    = gamepad.gamepad;
		this.red        = color.RED;
		this.green      = color.GREEN;
		this.blue       = color.BLUE;
		this.durationMS = Gamepad.LED_DURATION_CONTINUOUS;

	}

	@Override public void execute() {
		gamepad.setLedColor(red, green, blue, durationMS);
	}

	@Override public boolean isFinished() {
		return true;
	}
}
