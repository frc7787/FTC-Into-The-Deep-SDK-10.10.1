package org.firstinspires.ftc.teamcode.constants;

import android.annotation.SuppressLint;

public class Constants {

    public static class FileConstants {

        @SuppressLint("sdCardPath")
        public static final String ON_BOT_JAVA_PATH
                = "/sdcard/FIRST/java/src/org/firstinspires.ftc.teamcode/";

        public static final String CONSTANTS_FOLDER_PATH
                = ON_BOT_JAVA_PATH + "Constants/";
    }

    public static class TestConstants {
        public static int TEST_INT       = 0;
        public static double TEST_DOUBLE = 0.0;
        public static String TEST_STRING = "Hello, World";
    }

    public static class RobotConstants {
        public static double ROBOT_CHASSIS_LENGTH_INCHES = 14.2;
    }
}
