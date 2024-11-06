package org.firstinspires.ftc.teamcode.constants

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.*
import com.qualcomm.robotcore.hardware.*
import com.qualcomm.robotcore.hardware.DcMotor.*
import org.firstinspires.ftc.robotcore.external.navigation.*
import org.firstinspires.ftc.teamcode.constants.Constants.FileConstants.SD_CARD_PATH
import org.firstinspires.ftc.teamcode.constants.exceptions.*
import org.opencv.core.Scalar
import java.io.File
import java.lang.reflect.*
import java.util.Properties
import org.firstinspires.ftc.teamcode.constants.ConstantsType.*;
import java.io.FileInputStream
import kotlin.reflect.KClass
import com.qualcomm.robotcore.hardware.DcMotorSimple.*;
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.*
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.*
import org.firstinspires.ftc.teamcode.constants.ConstantsType.Companion.fieldToType
import java.io.IOException

class ConstantsLoader(telemetry: Telemetry? = null) {
    private val debugger = ConstantsDebugger(telemetry)

    private fun loadConstantsFiles(): Array<File> {
        val constantsDirectory = File("$SD_CARD_PATH/Constants")

        if (constantsDirectory.isFile) return arrayOf()

        val constantsFiles = constantsDirectory.listFiles()?: return arrayOf()
        val textFiles = constantsFiles.filter {
            it.name.substring(it.name.lastIndexOf('.') + 1) == ".txt"
        }
        return textFiles.toTypedArray()
    }

    fun load() {
        for (file in loadConstantsFiles()) {
            try {
                for (clazz in Constants::class.java.declaredClasses) {
                    if (isFieldLoadable(clazz.modifiers) && clazz.simpleName == file.name) {
                       loadClass(clazz, file.name)
                    }
                }
            } catch (ioException: IOException) {
                debugger.addIOException(ioException)
            }
        }
        if (debugger.telemetry != null) debugger.displayAll()
    }

    private fun loadClass(clazz: Class<*>, fileName: String) {
        val properties = Properties()

        properties.load(FileInputStream("$SD_CARD_PATH/Constants/$fileName"))

        for (field in clazz.fields) {
            if (!isFieldLoadable(field.modifiers)) continue
            loadField(field, properties)
        }
    }

    private fun loadField(field: Field, props: Properties) {
        val name = field.name

        if (!props.containsKey(name)) return

        when(fieldToType(field)) {
            SERVO_DIRECTION -> {
                field.set(name, loadEnum(name, props, Servo.Direction::class))
            }
            MOTOR_DIRECTION -> {
                field.set(name, loadEnum(name, props, Direction::class))
            }
            ZERO_POWER_BEHAVIOUR -> {
                field.set(name, loadEnum(name, props, ZeroPowerBehavior::class))
            }
            RUN_MODE -> {
                field.set(name, loadEnum(name, props, RunMode::class))
            }
            LOGO_FACING_DIRECTION -> {
                field.set(name, loadEnum(name, props, LogoFacingDirection::class))
            }
            USB_FACING_DIRECTION -> {
                field.set(name, loadEnum(name, props, UsbFacingDirection::class))
            }
            ANGLE_UNIT -> {
                field.set(name, loadEnum(name, props, AngleUnit::class))
            }
            DISTANCE_UNIT -> {
                field.set(name, loadEnum(name, props, DistanceUnit::class))
            }
            CURRENT_UNIT -> {
                field.set(name, loadEnum(name, props, CurrentUnit::class))
            }
            FLOAT -> {
                field.setFloat(name, props.getProperty(name).toFloat())
            }
            DOUBLE -> {
                field.setDouble(name, props.getProperty(name).toDouble())
            }
            BYTE -> {
                field.setByte(name, props.getProperty(name).toByte())
            }
            SHORT -> {
                field.setShort(name, props.getProperty(name).toShort())
            }
            INTEGER -> {
                field.setInt(name, props.getProperty(name).toInt())
            }
            LONG -> {
                field.setLong(name, props.getProperty(name).toLong())
            }
            BOOLEAN -> {
                field.setBoolean(name, props.getProperty(name).toBoolean())
            }
            CHAR -> {
                field.setChar(name, props.getProperty(name).toCharArray()[0])
            }
            STRING -> {
                field.set(name, props.getProperty(name))
            }
            POSE_2D -> {
                field.set(name, loadPose2D(name, props))
            }
            SCALAR -> {
                field.set(name, loadScalar(name, props))
            }
            UNKNOWN -> {}
        }
    }

    private fun loadEnum(key: String, properties: Properties, enum: KClass<*>): Any {
        val javaEnumClass = enum::class.java

        require(javaEnumClass.isEnum)

        val enumString = properties.getProperty(key).lowercase().replace("_", "")

        for (variant in javaEnumClass.enumConstants!!) {
            if (variant.toString().lowercase() == enumString) return variant
        }

        val reason = "It Could Not Be Parsed As ${javaEnumClass.simpleName}"
        val malformedPropertyException = MalformedPropertyException(key, reason, enumString)
        debugger.addMalformedPropertyException(malformedPropertyException)
        throw malformedPropertyException
    }

    private fun loadPose2D(key: String, properties: Properties): Pose2D {
        val pose2DString = properties.getProperty(key)
        val poseValues: Array<Double>

        try {
            poseValues = parseThreePartValue(pose2DString)
        } catch (_: NumberFormatException) {
            val reason = "It Could Not Be Parsed As Pose2D"
            val exception = MalformedPropertyException(key, reason, pose2DString)
            debugger.addMalformedPropertyException(exception)
            throw exception
        }

        return Pose2D(INCH, poseValues[0], poseValues[1], DEGREES, poseValues[2])
    }

    private fun loadScalar(key: String, properties: Properties): Scalar {
        val scalarString = properties.getProperty(key)
        val scalarValues: Array<Double>

        try {
            scalarValues = parseThreePartValue(scalarString)
        } catch (_: NumberFormatException) {
            val reason = "It Could Not Be Parsed As Scalar"
            val exception = MalformedPropertyException(key, reason, scalarString)
            debugger.addMalformedPropertyException(exception)
            throw exception
        }
        return Scalar(scalarValues[0], scalarValues[1], scalarValues[2])
    }

    private fun parseThreePartValue(value: String): Array<Double> {
        val indexOfOpenParentheses   = value.indexOf('(')
        val indexOfClosedParentheses = value.indexOf(')')
        val indexOfFirstComma        = value.indexOf(',')
        val indexOfSecondComma       = value.indexOf(',', indexOfFirstComma + 1)

        val valueOne   = value.substring(indexOfOpenParentheses + 1, indexOfFirstComma)
        val valueTwo   = value.substring(indexOfFirstComma + 1, indexOfSecondComma)
        val valueThree = value.substring(indexOfSecondComma + 1, indexOfClosedParentheses)

        return arrayOf(valueOne.toDouble(), valueTwo.toDouble(), valueThree.toDouble())
    }

    private fun isFieldLoadable(modifiers: Int): Boolean {
        return Modifier.isStatic(modifiers)
               && Modifier.isPublic(modifiers)
               && !Modifier.isFinal(modifiers)
    }
}