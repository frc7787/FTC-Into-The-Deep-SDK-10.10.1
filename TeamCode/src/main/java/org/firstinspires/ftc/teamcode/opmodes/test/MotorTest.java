package org.firstinspires.ftc.teamcode.opmodes.test;
import com.qualcomm.robotcore.eventloop.opmode.*;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem;;

@TeleOp(name = "Test - Arm Subsystem", group = "Test")
@Disabled
public final class MotorTest extends OpMode {
    private ArmSubsystem armSubsystem;

    @Override public void init() {
        armSubsystem = new ArmSubsystem(this);
    }

    @Override public void loop() {}
}
