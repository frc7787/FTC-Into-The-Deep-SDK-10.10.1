package org.firstinspires.ftc.teamcode.opmodes.test;

import static org.firstinspires.ftc.teamcode.vision.color.SampleColor.*;

import com.arcrobotics.ftclib.command.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.color.SampleColorDetector;
import org.openftc.easyopencv.*;

@TeleOp(name = "Test - Sample Color Detector", group = "Test")
public final class SampleColorDetectorTest extends CommandOpMode {
    private OpenCvCamera camera;
    private SampleColorDetector sampleDetector;
    private Gamepad currentGamepad, previousGamepad;

    @Override public void initialize() {
        sampleDetector  = new SampleColorDetector(YELLOW);
        currentGamepad  = new Gamepad();
        previousGamepad = new Gamepad();
        initializeWebcam();

        schedule(
                new RunCommand(this::displayInstructions),
                new RunCommand(this::toggleSampleColor),
                new RunCommand(telemetry::update)
        );
    }

    private void initializeWebcam() {
        int cameraMonitorViewId = hardwareMap
                .appContext
                .getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory
                .getInstance()
                .createWebcam(
                        hardwareMap.get(WebcamName.class, "Webcam 1"),
                        cameraMonitorViewId
                );

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                camera.setPipeline(sampleDetector);
            }

            @Override public void onError(int errorCode) {
                telemetry.addData("Failed to open camera due to error code", errorCode);
                telemetry.update();
            }
        });
    }

    private void displayInstructions() {
        telemetry.addLine("Press Left Bumper To Toggle Between Colors");
    }

    private void toggleSampleColor() {
        previousGamepad.copy(currentGamepad);
        currentGamepad.copy(gamepad1);

        if (currentGamepad.left_bumper && !previousGamepad.left_bumper) {
            switch (sampleDetector.detectionColor()) {
                case RED:
                    sampleDetector.setSampleColor(BLUE);
                    break;
                case BLUE:
                    sampleDetector.setSampleColor(YELLOW);
                    break;
                case YELLOW:
                    sampleDetector.setSampleColor(RED);
                    break;
            }
        }
    }
}
