package org.firstinspires.ftc.teamcode.constants

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.*
import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotor.*
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.navigation.*
import org.opencv.core.Scalar
import java.lang.reflect.Field

/**
 * Each variant represents a type that is supported by the [Loader] and
 * [ConstantsSaver] classes. Additionally, there is a "catch-all" variant UNKNOWN for
 * any type which is not currently supported.
 *
 * Currently, the list of supported types are as follows:
 *  * All primitive types and their wrappers.
 *  * All enums present on the [DcMotor] (excluding Manufacturer because
 *    it's read-only and should never be saved or loaded).
 *  * All enums present on [Servo]
 *  * The SDK flavour of Pose2D [Pose2D]
 *  * OpenCV Scalars
 *  * All enums present on IMU
 *  * AngleUnit and DistanceUnit
 *
 *
 */
internal enum class ConstantsType {
    // ---------------------------------------------------------------------------------------------
    // Primitives
    // ---------------------------------------------------------------------------------------------
    FLOAT, DOUBLE, BYTE, SHORT, INTEGER, LONG, BOOLEAN, CHAR,

    // ---------------------------------------------------------------------------------------------
    // Objects
    // ---------------------------------------------------------------------------------------------
    STRING, SERVO_DIRECTION, MOTOR_DIRECTION, ZERO_POWER_BEHAVIOUR,
    POSE_2D, RUN_MODE, SCALAR, LOGO_FACING_DIRECTION, USB_FACING_DIRECTION,
    ANGLE_UNIT, DISTANCE_UNIT, CURRENT_UNIT,
    UNKNOWN;

    companion object {
        /**
         * Converts the supplied field into its equivalent. SupportedType enum variant. This is useful
         * if you want to match a fields supported type, as you can't match on Class.
         * Note that both the primitive type and the wrapper class will return the same value. For
         * example both [Float] and float will return SupportedType.FLOAT.
         * @param field The field to return the supported type of
         * @return THe field, as a supported type.
         */
		@JvmStatic
		fun fieldToType(field: Field): ConstantsType {
            val fieldType = field.type

            if (fieldType == Float::class.javaPrimitiveType) return FLOAT
            if (fieldType == Double::class.javaPrimitiveType) return DOUBLE
            if (fieldType == Byte::class.javaPrimitiveType) return BYTE
            if (fieldType == Short::class.javaPrimitiveType) return SHORT
            if (fieldType == Int::class.javaPrimitiveType) return INTEGER
            if (fieldType == Long::class.javaPrimitiveType) return LONG
            if (fieldType == Boolean::class.javaPrimitiveType) return BOOLEAN
            if (fieldType == Char::class.javaPrimitiveType) return CHAR
            if (fieldType == String::class.java) return STRING
            if (fieldType == Servo.Direction::class.java) return SERVO_DIRECTION
            if (fieldType == Direction::class.java) return MOTOR_DIRECTION
            if (fieldType == ZeroPowerBehavior::class.java) return ZERO_POWER_BEHAVIOUR
            if (fieldType == SparkFunOTOS.Pose2D::class.java) return POSE_2D
            if (fieldType == Scalar::class.java) return SCALAR
            if (fieldType == LogoFacingDirection::class.java) return LOGO_FACING_DIRECTION
            if (fieldType == UsbFacingDirection::class.java) return USB_FACING_DIRECTION
            if (fieldType == AngleUnit::class.java) return ANGLE_UNIT
            if (fieldType == DistanceUnit::class.java) return DISTANCE_UNIT
            if (fieldType == CurrentUnit::class.java) return CURRENT_UNIT

            return UNKNOWN
        }
    }
}
