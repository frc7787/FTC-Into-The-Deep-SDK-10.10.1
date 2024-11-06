package org.firstinspires.ftc.teamcode.utility;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

public final class MotorGroup {
    DcMotor[] motors;

    public MotorGroup(@NonNull Direction direction, @NonNull DcMotor ... motors) {
       this.motors = motors;

       for (DcMotor motor : motors) {
           motor.setDirection(direction);
       }
    }

    public void setTargetPosition(int targetPosition) {
        for (DcMotor motor : motors) {
            motor.setTargetPosition(targetPosition);
        }
    }

    public void setMode(RunMode mode) {
        for (DcMotor motor : motors) {
            motor.setMode(mode);
        }
    }

    public void setPower(double power) {
       MotorUtility.setPowers(power, motors);
    }

    public int targetPosition() {
        return motors[0].getTargetPosition();
    }

    public int position() {
        return motors[0].getCurrentPosition();
    }

    public void setDirection(@NonNull Direction direction) {
       MotorUtility.setDirections(direction, motors);
    }

    public void reset() {
        MotorUtility.reset(motors);
    }
}
