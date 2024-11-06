package org.firstinspires.ftc.teamcode.constants

import com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.constants.Constants.FileConstants.*
import org.opencv.core.Scalar
import java.io.FileWriter
import java.lang.reflect.*
import org.firstinspires.ftc.teamcode.constants.ConstantsType.*;
import org.firstinspires.ftc.teamcode.constants.ConstantsType.Companion.fieldToType
import java.io.IOException

class ConstantsSaver(telemetry: Telemetry?) {
    private val debugger = ConstantsDebugger(telemetry)

    fun save() {
        for (clazz in Constants::class.java.declaredClasses) {
            if (!Modifier.isStatic(clazz.modifiers)) continue
            saveClass(clazz)
        }
        if (debugger.telemetry != null) debugger.displayAll()
    }

    private fun saveClass(clazz: Class<*>) {
        val className = clazz.simpleName
        val fileName  = "${SD_CARD_PATH}${className}.txt"
        val fields    = clazz.fields

        if (fields.isEmpty()) return

        try {
            val fileWriter = FileWriter(fileName)
            for (field in fields) saveField(fileWriter, field)
            fileWriter.flush()
        } catch (exception: IOException) {
            debugger.addIOException(exception)
        }
    }

    private fun saveField(fileWriter: FileWriter, field: Field) {
        if (!isSavable(field)) return

        val name = field.name
        val line = when (fieldToType(field)) {
            POSE_2D -> "$name=${pose2DString(field.get(null) as Pose2D)}"
            SCALAR            -> "$name=${scalarString(field.get(null) as Scalar)}"
            UNKNOWN           -> return
            else -> {
                "$name=${field.get(null)}"
            }
        }

        fileWriter.write(line + "\n")
    }

    private fun pose2DString(pose: Pose2D): String {
        return "Pose(${pose.x}, ${pose.y}, ${pose.h}"
    }

    private fun scalarString(scalar: Scalar): String {
        return "Scalar(${scalar.`val`[0]}, ${scalar.`val`[1]}, ${scalar.`val`[2]}"
    }

    private fun isSavable(field: Field): Boolean {
        val modifiers = field.modifiers

        return Modifier.isPublic(modifiers)
               && Modifier.isStatic(modifiers)
               && !Modifier.isFinal(modifiers)
    }
}