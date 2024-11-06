package org.firstinspires.ftc.teamcode.opmodes.test;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;
import static org.firstinspires.ftc.teamcode.drivers.GoBildaPinpointOdometry.*;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpointOdometry;

@TeleOp(name = "Test - Pin Point Sensor", group = "Test")
@Disabled
public final class PinPointTest extends OpMode {
    private GoBildaPinpointOdometry odometry;

    @Override public void init() {
        odometry = hardwareMap.get(GoBildaPinpointOdometry.class, "odometry");

        odometry.setEncoderResolution(FOUR_BAR_POD_TICKS_PER_MM);
        odometry.setOffsets(1200, 250);
        odometry.setEncoderDirections(EncoderDirection.REVERSED, EncoderDirection.REVERSED);

        odometry.resetPosAndIMU();

        telemetry.addData("X Offset", odometry.getXOffset());
        telemetry.addData("Y Offset", odometry.getYOffset());
        telemetry.addData("Version Number", odometry.getDeviceVersion());
        telemetry.update();
    }

    @Override public void loop() {
        odometry.update();
        Pose2D pose = odometry.getPosition();
        telemetry.addData("X", pose.getX(INCH));
        telemetry.addData("Y", pose.getY(INCH));
        telemetry.addData("H", pose.getHeading(DEGREES));
    }
}