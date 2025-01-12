package org.firstinspires.ftc.teamcode.subsystems.arm;

import androidx.annotation.NonNull;

import static org.firstinspires.ftc.teamcode.subsystems.arm.ArmConstants.*;

class ArmConversions {

    static int rotationDegreesToTicks(double degrees) {
        return (int) ((degrees + 11) * ROTATION_TICKS_PER_DEGREE) + 194;
    }

    static int extensionInchesToTicks(double inches) {
        return Math.max(0, (int) (inches * EXTENSION_TICKS_PER_INCH));
    }

    @NonNull static double[] cartesianToPolar(double horizontalInches, double verticalInches) {
        double extensionInches = Math.sqrt(
                Math.pow(horizontalInches, 2.0) +
                Math.pow(verticalInches, 2.0) +
                Math.pow(1.5, 2)
        ) - MIN_EXTENSION_INCHES;

        double rotationDegrees = Math.toDegrees(
                Math.atan(verticalInches / horizontalInches) -
                Math.atan(1.5 / (extensionInches + MIN_EXTENSION_INCHES))
        );

        return new double[] {rotationDegrees, extensionInches};
    }
}
