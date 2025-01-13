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

    @NonNull static double[] cartesianToPolar(double xInches, double yInches) {
        double extensionInches = Math.sqrt(
                Math.pow(xInches, 2.0) +
                Math.pow(yInches, 2.0) +
                Math.pow(1.5, 2)
        ) - MIN_EXTENSION_INCHES;

        double rotationDegrees = Math.toDegrees(
                Math.atan(yInches / xInches) -
                Math.atan(1.5 / (extensionInches + MIN_EXTENSION_INCHES))
        );

        return new double[] {rotationDegrees, extensionInches};
    }

    @NonNull static double[] polarToCartesian(double rotationDegrees, double extensionInches) {
        extensionInches += MIN_EXTENSION_INCHES;
        double thetaRadians = Math.toRadians(rotationDegrees) + Math.atan(1.5 / extensionInches);

        return new double[]{
                extensionInches * Math.cos(thetaRadians),
                extensionInches * Math.sin(thetaRadians)
        };
    }
}
