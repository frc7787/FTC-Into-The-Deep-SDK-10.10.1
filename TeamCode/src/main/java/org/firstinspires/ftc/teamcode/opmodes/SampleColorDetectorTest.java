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
    private final double KP = 0.004;
    private final double KD = 0.000014;

    private OpenCvCamera camera;
    private SampleColorDetector sampleDetector;
    //private TankDriveSubsystem tankDriveSubsystem;
    private PIDController turnController;

    @Override public void initialize() {
        sampleDetector     = new SampleColorDetector();
        //tankDriveSubsystem = new TankDriveSubsystem(this);

        // We keep KI at zero because we don't need to correct for error over time.
        turnController = new PIDController(KP, 0.0, KD);

        //register(tankDriveSubsystem);

        initializeWebcam();

        schedule(
                new RunCommand(this::centerOnLargestDetection)
        );
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

    private void centerOnLargestDetection() {
        SampleDetection largestDetection = sampleDetector.getLargestDetection(YELLOW);

        if (largestDetection == null) return;

        if (largestDetection.rect.area() == 1.0) {
            telemetry.addLine("Nothing Detected!");
            //tankDriveSubsystem.arcade(0.0, 0.0);
            return;
        }

        int centerPoint = (int) (largestDetection.rect.x + (largestDetection.rect.width / 2.0));

        int distFromCenter = centerPoint - 160;

        double power = -turnController.calculate(distFromCenter, 0);

        if (centerPoint <= (320 * 0.4)) {
            telemetry.addLine("Turning Left!");
            //tankDriveSubsystem.arcade(0.0, power);
        } else if (centerPoint >= (320 * 0.6)) {
            telemetry.addLine("Turning Right!");
            //tankDriveSubsystem.arcade(0.0, power);
        } else {
            telemetry.addLine("Centered!");
            //tankDriveSubsystem.arcade(0.0, 0.0);
        }

        telemetry.addData("Distance From Center", distFromCenter);
        telemetry.addData("Center Point", centerPoint);
        telemetry.addData("Power", power);

        telemetry.update();
    }
}
