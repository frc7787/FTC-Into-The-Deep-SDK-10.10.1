package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.ConstantsLoader;

@TeleOp(name = "Test")
@Disabled
public final class ConstantsLoaderTest extends OpMode {

    @Override public void init() {
        new ConstantsLoader(telemetry).load();
        telemetry.addLine("" + Constants.TestConstants.TEST_STRING);
    }

    @Override public void loop() {}
}
