package org.firstinspires.ftc.teamcode.subsystems.arm;

class ArmConstants {
    static final double EXTENSION_TICKS_PER_INCH = 394.0;
    static final double ROTATION_TICKS_PER_DEGREE = 29.0;
    static final double ROTATION_X_OFFSET_INCHES = 9;
    static final double ROTATION_Y_OFFSET_INCHES = -4.75;
    static final double ARM_STARTING_ANGLE_DEGREES = -11.0;

    static final double MIN_ROTATION_DEGREES = 0;
    static final double MAX_ROTATION_DEGREES = 90;
    static final double MIN_EXTENSION_INCHES = 11;
    static final double MAX_EXTENSION_INCHES = 45;

    static final double MAX_EXTENSION_POWER = 1.0;
    static final double MIN_EXTENSION_POWER = -1.0;
    static final double MAX_ROTATION_POWER  = 1.0;
    static final double MIN_ROTATION_POWER  = -1.0;

    static final double EXTENSION_HOMING_POWER = -0.8;
    static final double ROTATION_HOMING_POWER  = -0.6;

    static final int EXTENSION_POSITION_THRESHOLD = 150;
    static final int EXTENSION_NEGATIVE_THRESHOLD = (int) (2 / 1.5 * EXTENSION_POSITION_THRESHOLD);
    static final int ROTATION_POSITION_THRESHOLD  = 36;

    static final double DEFAULT_MANUAL_SPEED = 2; // In/Sec

    static final double[] START_POSITION_XY = new double[]{11.0, 0.0};
}
