package org.firstinspires.ftc.teamcode.opmodes.teleops.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.tankdrive.TankDriveSubsystem;

import static org.firstinspires.ftc.teamcode.opmodes.teleops.test.TankTest.TankDriveStyle.*;

@TeleOp(name = "Tank Drive", group = "Drive Only")
public final class TankTest extends OpMode {
    protected enum TankDriveStyle {
        CURVATURE,
        ARCADE,
        TANK
    }

    private TankDriveStyle tankDriveStyle;
    private boolean squareInputs;
    private TankDriveSubsystem tankDriveSubsystem;

    private Gamepad previousGamepad, currentGamepad;

    @Override public void init() {
        squareInputs = false;
        tankDriveStyle = CURVATURE;
        previousGamepad = new Gamepad();
        currentGamepad  = new Gamepad();
        tankDriveSubsystem = new TankDriveSubsystem(this);
    }

    @Override public void loop() {
        telemetry.addLine("Press Options (The One On The Left) To Display The Controls");
        if (gamepad1.options) {
            displayControls();
        } else {
            telemetry.addData("Squared Inputs", squareInputs);
            telemetry.addData("Drive Mode", tankDriveStyle);
        }

        previousGamepad.copy(currentGamepad);
        currentGamepad.copy(gamepad1);

        if (currentGamepad.cross && !previousGamepad.cross) squareInputs = !squareInputs;

        switch (tankDriveStyle) {
           case TANK:
               double leftPower = gamepad1.left_stick_y * -1.0;
               double rightPower = gamepad1.right_stick_y * -1.0;

               if (squareInputs) {
                   leftPower *= Math.abs(leftPower);
                   rightPower *= Math.abs(rightPower);
               }

               tankDriveSubsystem.tank(leftPower, rightPower);
               break;
           case ARCADE:
               double drivePower = gamepad1.left_stick_y * -1.0;
               double turnPower = gamepad1.left_stick_x * -1.0;

               if (squareInputs) {
                   drivePower *= Math.abs(drivePower);
                   turnPower *= Math.abs(turnPower);
               }

               tankDriveSubsystem.arcade(drivePower, turnPower);
               break;
           case CURVATURE:
               double forwardsPower = gamepad1.left_trigger;
               double reversePower = gamepad1.right_trigger * -1.0;
               double rotationPower = gamepad1.left_stick_x;

               if (squareInputs) {
                   forwardsPower *= Math.abs(forwardsPower);
                   reversePower *= Math.abs(reversePower);
                   rotationPower *= Math.abs(rotationPower);
               }

               tankDriveSubsystem.curvature(
                       forwardsPower,
                       reversePower,
                       rotationPower,
                       gamepad1.left_bumper
               );
               break;
        }
    }

    private void displayControls() {
        telemetry.addLine("Press X To Toggle Squared Inputs");

        switch (tankDriveStyle) {
            case TANK:
                telemetry.addLine("Use The Left Stick Y (Up/Down) To Control The Left Wheels");
                telemetry.addLine("Use The Right Stick Y (Up/Down) To Control The Right Wheels");
                break;
            case ARCADE:
                telemetry.addLine("Use The Left Stick Y (Up/Down) To Move Forwards/Backwards");
                telemetry.addLine("Use The Left Stick X (Left/Right) To Turn");
                break;
            case CURVATURE:
                telemetry.addLine("Use The Left Trigger To Go Forwards");
                telemetry.addLine("Use The Right Trigger To Go In Reverse");
                telemetry.addLine("Use The Left Stick X (Left/Right) To Turn");
                telemetry.addLine("Hold Left Bumper To Allow Turning In Place");
                break;
        }
    }
}
