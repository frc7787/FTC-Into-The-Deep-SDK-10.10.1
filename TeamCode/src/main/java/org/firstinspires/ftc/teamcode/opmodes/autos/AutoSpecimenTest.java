package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous (name = "AutoSpecimenTest")
public class AutoSpecimenTest extends LinearOpMode {
    Pose2d initialPose = new Pose2d(10, -62, Math.PI/2);

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive.Builder(hardwareMap)
                .setPose(initialPose)
                .build();

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToY(-45)
                .waitSeconds(1.0)
                .splineToSplineHeading(new Pose2d(35, -33, 0), 0)
                .splineToLinearHeading(new Pose2d(40, -13, 0), 0)
                .splineToConstantHeading(new Vector2d(47.5, -57), -Math.PI / 2);

        while (!isStopRequested() && !opModeIsActive()) {
            Pose2d position = drive.pose;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

        waitForStart();

        Action trajectoryAction;

        trajectoryAction = tab1.build();

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryAction
                ) // end sequentialaction
        ); // end actions.runblocking




    } // end public void runOpMode


} // end public class AutoSpecimenTest