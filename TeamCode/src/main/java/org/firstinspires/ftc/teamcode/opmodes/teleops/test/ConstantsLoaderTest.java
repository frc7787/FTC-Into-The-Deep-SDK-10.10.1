package org.firstinspires.ftc.teamcode.opmodes.teleops.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constants.ConstantsLoader;
import static org.firstinspires.ftc.teamcode.constants.Constants.TestConstants.*;

@TeleOp(name = "Test")
@Disabled
public final class ConstantsLoaderTest extends OpMode {

    @Override public void init() {
        new ConstantsLoader(telemetry).load();
        telemetry.addData("Test Boolean", TEST_BOOLEAN);
        telemetry.addData("Test Double", TEST_DOUBLE);
        telemetry.addData("Test Float", TEST_FLOAT);
        telemetry.addData("Test Int", TEST_INT);
        telemetry.addData("Test Byte", TEST_BYTE);
        telemetry.addData("Test Short", TEST_SHORT);
        telemetry.addData("Test Long", TEST_LONG);
        telemetry.addData("Test String", TEST_STRING);
        telemetry.addData("Test Enum", TEST_ENUM);
        telemetry.addData("Test Pose", TEST_POSE);
        telemetry.update();
    }

    @Override public void loop() {}
}
