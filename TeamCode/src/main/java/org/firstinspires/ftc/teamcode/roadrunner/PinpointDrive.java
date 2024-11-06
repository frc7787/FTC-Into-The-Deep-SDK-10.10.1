package org.firstinspires.ftc.teamcode.roadrunner;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.roadrunner.messages.PoseMessage;

/**
 * Experimental extension of MecanumDrive that uses the Gobilda Pinpoint sensor for localization.
 * <p>
 * Released under the BSD 3-Clause Clear License by j5155 from 12087 Capital City Dynamics
 * Portions of this code made and released under the MIT License by Gobilda (Base 10 Assets, LLC)
 * Unless otherwise noted, comments are from Gobilda
 */
public class PinpointDrive extends MecanumDrive {
    public static class Params {
        public double X_OFFSET_INCHES = 4.75;
        public double Y_OFFSET_INCHES = 1.00;

        public double encoderResolution = GoBildaPinpointDriverRR.goBILDA_4_BAR_POD;

        public GoBildaPinpointDriver.EncoderDirection xDirection
                = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        public GoBildaPinpointDriver.EncoderDirection yDirection
                = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    }

    public static final Params PARAMS = new Params();
    public final GoBildaPinpointDriverRR pinpoint;
    private Pose2d lastPinpointPose = pose;

    /**
     * Creates a new pinpoint drive (Extension of MecanumDrive)
     * @param hardwareMap The hardwareMap to get the motors and pinpoint sensor from
     * @param pose The position to start at
     */
    public PinpointDrive(HardwareMap hardwareMap, Pose2d pose) {
        super(hardwareMap, pose);
        FlightRecorder.write("PINPOINT_PARAMS",PARAMS);
        pinpoint = hardwareMap.get(GoBildaPinpointDriverRR.class, "pinpoint");

        pinpoint.setOffsets(
                DistanceUnit.MM.fromInches(PARAMS.X_OFFSET_INCHES),
                DistanceUnit.MM.fromInches(PARAMS.Y_OFFSET_INCHES)
        );
        pinpoint.setEncoderResolution(PARAMS.encoderResolution);
        pinpoint.setEncoderDirections(PARAMS.xDirection, PARAMS.yDirection);
        pinpoint.resetPosAndIMU();

        // Wait for the IMU to reset itself
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        pinpoint.setPosition(pose);
    }

    /**
     * Updates the current estimated position
     * @return The current estimated position & velocity of the robot
     */
    @Override public PoseVelocity2d updatePoseEstimate() {
        // Bad solution ; Should fix later
        if (lastPinpointPose != pose) pinpoint.setPosition(pose);

        pinpoint.update();
        pose = pinpoint.getPositionRR();
        lastPinpointPose = pose;

        // RR standard
        poseHistory.add(pose);
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        FlightRecorder.write("ESTIMATED_POSE", new PoseMessage(pose));
        FlightRecorder.write("PINPOINT_RAW_POSE",new FTCPoseMessage(pinpoint.getPosition()));
        FlightRecorder.write("PINPOINT_STATUS",pinpoint.getDeviceStatus());

        return pinpoint.getVelocityRR();
    }

    /**
     * Log message containing the current pose, and time ; For debugging purposes
     */
    public static final class FTCPoseMessage {
        public long timestamp;
        public double x;
        public double y;
        public double heading;

        /**
         * Creates a new debug message containing the current pose, and time ; For debugging
         * purposes.
         * @param pose The current position of the robot
         */
        public FTCPoseMessage(@NonNull Pose2D pose) {
            this.timestamp = System.nanoTime();
            this.x         = pose.getX(INCH);
            this.y         = pose.getY(INCH);
            this.heading   = pose.getHeading(RADIANS);
        }
    }
}
