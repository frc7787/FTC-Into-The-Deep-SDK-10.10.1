package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.vision.SampleColor.*;

import com.arcrobotics.ftclib.command.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.tankdrive.TankDriveSubsystem;
import org.firstinspires.ftc.teamcode.utility.PIDController;
import org.firstinspires.ftc.teamcode.vision.SampleColorDetector;
import org.firstinspires.ftc.teamcode.vision.SampleDetection;
import org.openftc.easyopencv.*;

@TeleOp(name = "Test - Sample Color Detector", group = "Test")
public final class SampleColorDetectorTest extends CommandOpMode {
    private OpenCvCamera camera;
    private SampleColorDetector sampleDetector;

    @Override public void initialize() {
        sampleDetector = new SampleColorDetector(YELLOW);
        initializeWebcam();
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
}
