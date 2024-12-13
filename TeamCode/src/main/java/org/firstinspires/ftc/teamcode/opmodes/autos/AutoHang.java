package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantFunction;
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
public class AutoHang extends LinearOpMode {
    private final Pose2d initialPose = new Pose2d(8, -62, Math.PI / 2);

    @Override public void runOpMode() {
        Arm arm = new Arm(this);
        ElapsedTime elapsedTime = new ElapsedTime();

        MecanumDrive drive = new MecanumDrive.Builder(hardwareMap)
                .setPose(initialPose)
                .build();

        TrajectoryActionBuilder startToBarBuilder = drive.actionBuilder(initialPose)
                .setTangent(Math.PI / 2)
                .splineTo(new Vector2d(2, -27), Math.PI/2);

        TrajectoryActionBuilder barToObservationZoneBuilder = startToBarBuilder.endTrajectory().fresh()
                .setTangent(-Math.PI / 2)
                .splineToLinearHeading(new Pose2d(26, -40, Math.PI), Math.PI / 2)
                .setTangent(Math.PI / 2)
                .splineToSplineHeading(new Pose2d(30, -21, -Math.PI /2), Math.PI/2)
                .setTangent(Math.PI/2)
                .splineToConstantHeading(new Vector2d(47, -10), 0)
                .setTangent(-Math.PI/2)
                .splineToConstantHeading(new Vector2d(47, -56), -Math.PI / 2)

                //push second sample to zone
                .setTangent(Math.PI / 2)
                .splineToSplineHeading(new Pose2d(47, -20, -Math.PI /2), Math.PI / 4)
                .setTangent(Math.PI/4)
                .splineToLinearHeading(new Pose2d(50, -15, -Math.PI /4), Math.PI / 2);
        // ends before contacting second sample


        TrajectoryActionBuilder sample2ToObservationBuilder = barToObservationZoneBuilder.endTrajectory().fresh()
                .setTangent(-Math.PI/4)
                .splineTo(new Vector2d(58, -28), -Math.PI/2)
                .setTangent(-Math.PI/2)
                .splineToSplineHeading(new Pose2d(58, -60, -Math.PI /2), -Math.PI / 2)
                .waitSeconds(1)
                .setTangent(-Math.PI/2)
                .lineToY(-63);

        Action observationZoneToBar = sample2ToObservationBuilder.endTrajectory().fresh()

                .setTangent(Math.PI/2)
                .splineToSplineHeading(new Pose2d(0, -30, Math.PI /1.999), Math.PI / 2)
                .build();

        Action barToObservationZone = barToObservationZoneBuilder.build();
        Action sample2ToObservationZone = sample2ToObservationBuilder.build();
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

        while (!arm.isAtPosition() && elapsedTime.seconds() < 3.0) {
            arm.update();
        }
        arm.stop();

        Actions.runBlocking(barToObservationZone);

        arm.setTargetPositionInches(9.8, 6);
        arm.setMaxSpeed(1.0);
        elapsedTime.reset();

        while (!arm.isAtPosition() && elapsedTime.seconds() < 1.0) {
            arm.update();
            arm.debugPosition();
            arm.update();
        }
        arm.stop();
        Actions.runBlocking(sample2ToObservationZone);
        Actions.runBlocking(observationZoneToBar);


    }
}
