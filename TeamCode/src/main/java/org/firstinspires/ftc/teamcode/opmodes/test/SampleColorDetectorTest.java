package org.firstinspires.ftc.teamcode.opmodes.test;

import static org.firstinspires.ftc.teamcode.vision.color.SampleColor.*;

import com.arcrobotics.ftclib.command.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.color.SampleColor;
import org.firstinspires.ftc.teamcode.vision.color.SampleColorDetector;
import org.firstinspires.ftc.teamcode.vision.color.SampleDetection;
import org.opencv.core.Rect;
import org.openftc.easyopencv.*;

@TeleOp(name = "Test - Sample Color Detector", group = "Test")
public final class SampleColorDetectorTest extends CommandOpMode {
    private OpenCvCamera camera;
    private SampleColorDetector sampleDetector;
    private Gamepad currentGamepad, previousGamepad;
    private SampleColor detectionColor;

    @Override public void initialize() {
        detectionColor = YELLOW;

        sampleDetector  = new SampleColorDetector(detectionColor);
        currentGamepad  = new Gamepad();
        previousGamepad = new Gamepad();
        initializeWebcam();

        schedule(
                new RunCommand(this::displayInstructions),
                new RunCommand(telemetry::addLine),
                new RunCommand(this::displayLargestDetectionInformation),
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

    private void displayLargestDetectionInformation() {
        SampleDetection largestDetection = sampleDetector.getLargestDetection(detectionColor);

        if (largestDetection == null) {
            telemetry.addLine("Nothing Detected");
        } else {
            Rect largestDetectionRectangle = largestDetection.rect;
            telemetry.addData("Largest Detection Area", largestDetectionRectangle.area());
            telemetry.addData("Largest Detection Width", largestDetectionRectangle.width);
            telemetry.addData("Largest Detection Height", largestDetectionRectangle.height);
            telemetry.addData("Largest Detection X", largestDetectionRectangle.x);
            telemetry.addData("Largest Detection Y", largestDetectionRectangle.y);
        }
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
                    detectionColor = BLUE;
                    break;
                case BLUE:
                    detectionColor = YELLOW;
                    break;
                case YELLOW:
                    detectionColor = RED;
                    break;
            }
        }

        sampleDetector.setSampleColor(detectionColor);
    }
}
