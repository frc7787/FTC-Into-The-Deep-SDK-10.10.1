package org.firstinspires.ftc.teamcode.constants

import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.constants.exceptions.MalformedPropertyException
import org.firstinspires.ftc.teamcode.utility.Debugger
import java.io.IOException

class ConstantsDebugger(telemetry: Telemetry?) : Debugger(telemetry) {
    fun addMalformedPropertyException(exception: MalformedPropertyException) {
        val issue = """
            Field ${exception.name} Cannot Be Loaded
            Reason: Value ${exception.value} is Malformed Because ${exception.reason} 
        """.trimIndent()
        output.add(issue)
    }

    fun addIOException(ioException: IOException) {
        val issue = """
            IOException Occurred When Loading Constants
            Reason: ${ioException.message}
        """.trimIndent()
        output.add(issue)
    }
}
