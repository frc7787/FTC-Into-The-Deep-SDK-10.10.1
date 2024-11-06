package org.firstinspires.ftc.teamcode.constants.exceptions

class MalformedPropertyException(
    val name: String,
    val reason: String,
    val value: String
) : Exception("Malformed Property Exception $name")


