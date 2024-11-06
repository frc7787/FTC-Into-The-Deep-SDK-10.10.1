package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.teamcode.vision.SampleColor.*;
import static org.firstinspires.ftc.teamcode.constants.Constants.VisionConstants.*;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

/**
 * OpenCV pipeline to detect the different colored samples for the FTC 2024-2025 game Into The Deep
 * @Author Tyler Stocks
 */
public final class SampleColorDetector extends OpenCvPipeline {
    private final Mat hsvMat          = new Mat(),
                      threshold0      = new Mat(),
                      threshold1      = new Mat(),
                      hierarchy       = new Mat(),
                      cvErodeKernel   = new Mat(),
                      thresholdOutput = new Mat(),
                      erodeOutput     = new Mat(),
                      dilateOutput    = new Mat(),
                      cvDilateKernel  = new Mat();

    private ArrayList<SampleDetection> sampleDetections = new ArrayList<>();
    private SampleDetection largestDetection, largestRedSample, largestYellowSample, largestGreenSample;

    @Override public Mat processFrame(Mat input) {
        sampleDetections = new ArrayList<>();

        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        detectSample(input, YELLOW);

        return input;
    }

    private synchronized void detectSample(@NonNull Mat input, @NonNull SampleColor color) {
        switch (color) {
            case RED:
                // Check if the image is in range, then adds the ranges together
                Core.inRange(hsvMat, LOW_HSV_RANGE_RED_ONE, HIGH_HSV_RANGE_RED_ONE, threshold0);
                Core.inRange(hsvMat, LOW_HSV_RANGE_RED_TWO, HIGH_HSV_RANGE_RED_TWO, threshold1);
                Core.add(threshold0, threshold1, thresholdOutput);
                break;
            case BLUE:
                // Checks if the image is in range
                Core.inRange(hsvMat, LOW_HSV_RANGE_BLUE, HIGH_HSV_RANGE_BLUE, thresholdOutput);
                break;
            case YELLOW:
                Core.inRange(hsvMat, LOW_HSV_RANGE_YELLOW, HIGH_HSV_RANGE_YELLOW, thresholdOutput);
                break;
        }

        // Erode to remove noise
        Imgproc.erode(
                thresholdOutput,
                erodeOutput,
                cvErodeKernel,
                CV_ANCHOR,
                ERODE_PASSES,
                Core.BORDER_CONSTANT,
                CV_BORDER_VALUE
        );

        Imgproc.dilate(
               erodeOutput,
               dilateOutput,
               cvDilateKernel,
               CV_ANCHOR,
               4,
               CV_BORDER_TYPE,
               CV_BORDER_VALUE
        );

        // Finds the contours of the image
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(
                dilateOutput,
                contours,
                hierarchy,
                Imgproc.RETR_TREE,
                Imgproc.CHAIN_APPROX_SIMPLE
        );

        // Creates bounding rectangles along all of the detected contours
        MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contours.size()];
        Rect[] boundRect = new Rect[contours.size()];
        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(
                    new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
        }

        for (Rect rect : boundRect) {
            Imgproc.rectangle(input, rect, BOUNDING_RECTANGLE_COLOR);
            sampleDetections.add(new SampleDetection(rect, color));
        }
    }

    @Nullable public synchronized SampleDetection getLargestDetection(@NonNull SampleColor color) {
        if (sampleDetections.isEmpty()) return null;

        SampleDetection largestDetection = new SampleDetection(new Rect(0,0,1,1), color);

        for (SampleDetection detection : sampleDetections) {
            if (detection.color != color) continue;
            if (detection.rect.area() <= largestDetection.rect.area()) continue;

            largestDetection = detection;
        }

        return largestDetection;
    }
}