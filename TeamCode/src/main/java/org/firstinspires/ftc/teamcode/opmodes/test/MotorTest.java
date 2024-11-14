package org.firstinspires.ftc.teamcode.opmodes.test;
import com.qualcomm.robotcore.eventloop.opmode.*;
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;;

@TeleOp(name = "Test - Arm Subsystem", group = "Test")
@Disabled
public final class MotorTest extends OpMode {
    private Arm armSubsystem;

    @Override public void init() {
        armSubsystem = new Arm(this);
    }

    @Override public void loop() {}
}
