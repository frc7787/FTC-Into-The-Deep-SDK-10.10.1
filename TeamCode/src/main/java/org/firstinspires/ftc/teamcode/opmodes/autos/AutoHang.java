package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;

@Autonomous (name = "AutoSpecimenTest")
public class AutoHang extends LinearOpMode {
    Pose2d initialPose = new Pose2d(8, -62, Math.PI/2);

    @Override
    public void runOpMode() {
        Arm arm = new Arm(this);
        ElapsedTime elapsedTime = new ElapsedTime();

        MecanumDrive drive = new MecanumDrive.Builder(hardwareMap)
                .setPose(initialPose)
                .build();

        Action startToBar = drive.actionBuilder(initialPose)
                .lineToY(-27)
                .build();

        while (!isStopRequested() && !opModeIsActive()) {
            Pose2d position = drive.pose;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

        waitForStart();
        elapsedTime.reset();

        while (arm.state() == Arm.ArmState.HOMING) {
            telemetry.addData("State", arm.state());
            telemetry.addLine("Homing");
            telemetry.update();
            arm.update();
        }

        arm.setTargetPositionInchesRobotCentric(1.5, 24.5);
        elapsedTime.reset();

        while (!arm.isAtPosition() || elapsedTime.seconds() > 5.0) {
            telemetry.addLine("Moving To Position");
            telemetry.update();
            arm.update();
        }
        arm.stop();

        Actions.runBlocking(startToBar);

        arm.setExtensionTargetPosition(3);
        arm.setMaxSpeed(0.4);
        elapsedTime.reset();

        while (!arm.isAtPosition() || elapsedTime.seconds() > 3.0) {
            arm.update();
        }
        arm.stop();
    }
}