package org.firstinspires.ftc.teamcode.opmodes.teleops.test;

import static org.firstinspires.ftc.teamcode.constants.Constants.TestConstants.TEST_DOUBLE;
import static org.firstinspires.ftc.teamcode.constants.Constants.TestConstants.TEST_INT;
import static org.firstinspires.ftc.teamcode.constants.Constants.TestConstants.TEST_STRING;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constants.ConstantsLoader;

@TeleOp(name = "Test - Constants Loader", group = "Test")
public class ConstantsLoaderTest extends OpMode {

    @Override public void init() {
        new ConstantsLoader(telemetry).load();
    }

    @Override public void loop() {
        telemetry.addData("Test Int", TEST_INT);
        telemetry.addData("Test Double", TEST_DOUBLE);
        telemetry.addData("Test String", TEST_STRING);
    }
}
