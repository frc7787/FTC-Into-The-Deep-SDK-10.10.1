package org.firstinspires.ftc.teamcode.subsystems.elevator;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ElevatorSubsystem extends SubsystemBase {
    private DcMotor elevatorMotor;

    public double power = 0.0;

    public ElevatorSubsystem(@NonNull HardwareMap hardwareMap) {
        elevatorMotor       = hardwareMap.get(DcMotor.class, "elevatorMotor");
    }

    @Override public void periodic() {
       elevatorMotor.setPower(power);
    }
}
