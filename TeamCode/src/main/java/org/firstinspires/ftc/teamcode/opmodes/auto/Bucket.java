package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;

@Autonomous
public class Bucket extends LinearOpMode {
    private final Pose2d initialPose = new Pose2d(-8, -62, Math.PI / 2);

    @Override public void runOpMode() {
        Arm arm = new Arm(this);
        ElapsedTime elapsedTime = new ElapsedTime();

        MecanumDrive drive = new MecanumDrive.Builder(hardwareMap)
                .setPose(initialPose)
                .build();

        TrajectoryActionBuilder startToBarBuilder = drive.actionBuilder(initialPose)
                .setTangent(Math.PI / 2)
                .splineTo(new Vector2d(-2, -27), Math.PI/2);

        Action barToBuckets = startToBarBuilder.endTrajectory().fresh()
                .setTangent(-Math.PI / 2)
                .splineToLinearHeading(new Pose2d(-33, -36, -Math.PI/2), Math.PI / 2)
                .setTangent(Math.PI / 2)
                .splineToConstantHeading(new Vector2d(-45, -10), Math.PI)
                .setTangent(-Math.PI/2)
                .splineToSplineHeading(new Pose2d(-46, -60.5, -Math.PI /2), -Math.PI / 2)
                .setTangent(Math.PI / 2)
                .splineToLinearHeading(new Pose2d(-46, -16, -3*Math.PI/4), 3*Math.PI / 4)
                .setTangent(3*Math.PI/4)
                .splineToLinearHeading(new Pose2d(-53, -9, -Math.PI /2), Math.PI / 2)
                .setTangent(-Math.PI/2)
                .splineToLinearHeading(new Pose2d(-53, -60, -Math.PI /2), -Math.PI / 2)
                .setTangent(Math.PI/2)
                .splineToSplineHeading(new Pose2d(-53, -16, -Math.PI /2), Math.PI / 2)
                .setTangent(Math.PI/2)
                .splineToConstantHeading(new Vector2d(-60, -8), Math.PI/2)
                .setTangent(-Math.PI/2)
                .splineToSplineHeading(new Pose2d(-60, -50, -Math.PI /2), -Math.PI / 2)
                .setTangent(Math.PI/2)
                .splineToLinearHeading(new Pose2d(-36, -12, 0), 0)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-20, -12, 0), 0)
                .build();

        Action startToBar = startToBarBuilder.build();

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



        Actions.runBlocking(barToBuckets);

        arm.setTargetPositionInchesRobotCentric(8, 5);
        arm.setMaxSpeed(1.0);

        elapsedTime.reset();

        while (!arm.isAtPosition() || elapsedTime.seconds() > 5.0) {
            arm.update();
        }
    }
}
