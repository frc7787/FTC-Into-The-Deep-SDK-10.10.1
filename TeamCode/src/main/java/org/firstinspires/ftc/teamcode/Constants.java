package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver.EncoderDirection;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.opencv.core.Core;
import org.opencv.core.Point;
import org.opencv.core.Scalar;

public final class Constants {

    public static class FileConstants {
        @SuppressLint("sdCardPath")
        public static final String SD_CARD_PATH
                = "/sdcard/FIRST/java/src/org/firstinspires/ftc/teamcode/";
    }

    public static class ArmConstants {
        public static int EXTENSION_LIMIT        = 0; //TODO measure this value
        public static int ROTATION_HOME_POSITION = 0; //TODO measure this value

        public static double DEFAULT_ROTATION_POWER  = 0.6;
        public static double DEFAULT_EXTENSION_POWER = 0.6;
    }

    public static class DrivebaseConstants {
        public static String FRONT_LEFT_DRIVE_MOTOR_NAME  = "frontLeftDriveMotor";
        public static String FRONT_RIGHT_DRIVE_MOTOR_NAME = "frontRightDriveMotor";
        public static String BACK_LEFT_DRIVE_MOTOR_NAME   = "backLeftDriveMotor";
        public static String BACK_RIGHT_DRIVE_MOTOR_NAME  = "backRightDriveMotor";
        public static String ODOMETRY_NAME                = "pinpoint";
        public static double ODOMETRY_X_OFFSET_MM = 0;
        public static double ODOMETRY_Y_OFFSET_MM = 0;
        public static EncoderDirection X_ENCODER_DIRECTION = EncoderDirection.FORWARD;
        public static EncoderDirection Y_ENCODER_DIRECtiON = EncoderDirection.FORWARD;
    }

    public static class TestConstants {
        public static String TEST_STRING    = "Hello, World";
        public static boolean TEST_BOOLEAN  = true;
        public static int TEST_INT          = 1384383;
        public static byte TEST_BYTE        = 111;
        public static short TEST_SHORT      = 24589;
        public static long TEST_LONG        = 99999999999999L;
        public static double TEST_DOUBLE    = 0.342843;
        public static float TEST_FLOAT      = 0.438f;
        public static CurrentUnit TEST_ENUM = CurrentUnit.AMPS;
        public static Pose2D TEST_POSE      = new Pose2D(INCH, 0, 0, DEGREES, 0);
    }

    public static class AprilTagConstants {
        public static int GAIN          = 0;
        public static int EXPOSURE_MS   = 2;
        public static int WHITE_BALANCE = 4000;
    }

    public static class VisionConstants {
        public static int ERODE_PASSES = 5;

        public static volatile Scalar BOUNDING_RECTANGLE_COLOR = new Scalar(255, 0, 0);

        public static Scalar LOW_HSV_RANGE_BLUE  = new Scalar(97, 50, 0);
        public static Scalar HIGH_HSV_RANGE_BLUE = new Scalar(125, 255, 255);

        public static Scalar LOW_HSV_RANGE_RED_ONE  = new Scalar(170, 50, 50);
        public static Scalar HIGH_HSV_RANGE_RED_ONE = new Scalar(180, 255, 255);

        public static Scalar LOW_HSV_RANGE_RED_TWO  = new Scalar(0, 50, 50);
        public static Scalar HIGH_HSV_RANGE_RED_TWO = new Scalar(10, 255, 255);

        public static Scalar LOW_HSV_RANGE_YELLOW = new Scalar(25, 50, 0);
        public static Scalar HIGH_HSV_RANGE_YELLOW = new Scalar(35, 255, 255);

        public static final Point CV_ANCHOR        = new Point(-1, -1);
        public static final Scalar CV_BORDER_VALUE = new Scalar(-1);
        public static final int CV_BORDER_TYPE     = Core.BORDER_CONSTANT;
    }
}
