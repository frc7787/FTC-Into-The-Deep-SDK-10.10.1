package org.firstinspires.ftc.teamcode.opmodes.test;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.tankdrive.TankDriveSubsystem;
import org.openftc.easyopencv.*;

@TeleOp(name = "Test - Sample Color Detector", group = "Test")
public final class SampleColorDetectorTest extends OpMode {
    private final double KP = 0.004;
    private final double KD = 0.000014;

    private OpenCvCamera camera;
    private SampleDetector sampleDetector;
    private TankDriveSubsystem tankDriveSubsystem;

    @Override public void init() {
        sampleDetector     = new SampleDetector();
        initializeWebcam();
    }

    private void initializeWebcam() {
        int cameraMonitorViewId = hardwareMap
                .appContext
                .getResources()
                .getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName()
                );

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

    @Override public void loop() {

    }
}

