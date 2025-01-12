package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmodes.auto.actions.HomeArmAction;
import org.firstinspires.ftc.teamcode.opmodes.auto.actions.MoveArmToPositionAction;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;

@Autonomous
public class Bar extends LinearOpMode {
    private final Pose2d initialPose = new Pose2d(8, -62, Math.PI / 2);

    private final double HIGH_BAR_VERTICAL_INCHES = 24.5;
    private final double HIGH_BAR_HORIZONTAL_INCHES = 1.5;
    private final double CLIPPING_VERTICAL_INCHES = 20.5;
    private final double WALL_VERTICAL_INCHES = 6.0;
    private final double WALL_HORIZONTAL_INCHES = 9.8;
    private final double PICKUP_VERTICAL_INCHES = 8.5;

    @Override public void runOpMode() {
        Arm arm = new Arm(this);
        ElapsedTime elapsedTime = new ElapsedTime();

        MecanumDrive drive = new MecanumDrive.Builder(hardwareMap)
                .setPose(initialPose)
                .build();

        TrajectoryActionBuilder startToBarAction = drive.actionBuilder(initialPose)
                .setTangent(Math.PI / 2)
                .splineTo(new Vector2d(4, -27), Math.PI/2);

        TrajectoryActionBuilder pushFirstTwoSamplesIntoToObservationZoneAction = startToBarAction.endTrajectory().fresh()
                // To First Sample
                .setTangent(-Math.PI / 2)
                .splineToLinearHeading(new Pose2d(26, -36, Math.PI), Math.PI / 2)
                .setTangent(Math.PI / 2)
                .splineToSplineHeading(new Pose2d(30, -21, -Math.PI /2), Math.PI/2)
                .setTangent(Math.PI/2)
                .splineToConstantHeading(new Vector2d(47, -10), 0)
                .setTangent(-Math.PI/2)
                .splineToConstantHeading(new Vector2d(47, -56), -Math.PI / 2)
                // Push Sample Into Zone
                .setTangent(Math.PI / 2)
                .splineToSplineHeading(new Pose2d(47, -20, -Math.PI /2), Math.PI / 4)
                .setTangent(Math.PI/4)
                .splineToLinearHeading(new Pose2d(50, -15, -Math.PI /4), Math.PI / 2)
                // Push Second Sample Into Zone
                .setTangent(-Math.PI/4)
                .splineTo(new Vector2d(58, -28), -Math.PI/2)
                .setTangent(-Math.PI/2)
                .splineToSplineHeading(new Pose2d(58, -56, -Math.PI /2), -Math.PI / 2);

        TrajectoryActionBuilder attachToFirstSample = pushFirstTwoSamplesIntoToObservationZoneAction.endTrajectory().fresh()
                .waitSeconds(0.3)
                .setTangent(-Math.PI/2)
                .lineToY(-67, null, new ProfileAccelConstraint(-70.0, 70.0));

        TrajectoryActionBuilder firstSamplePickupToBar = attachToFirstSample.endTrajectory().fresh()
                .waitSeconds(0.3)
                .setTangent(-Math.PI/2)
                .lineToY(-67, null, new ProfileAccelConstraint(-70.0, 70.0))
                .setTangent(Math.PI/2)
                .splineToSplineHeading(new Pose2d(-2, -30, Math.PI /1.999), Math.PI / 2);

        TrajectoryActionBuilder barToSecondSamplePickup = firstSamplePickupToBar.endTrajectory().fresh()
                .setTangent(-Math.PI/2)
                .splineToSplineHeading(new Pose2d(58, -56, -Math.PI / 2), -Math.PI / 2);

        TrajectoryActionBuilder attachToSecondSample = barToSecondSamplePickup.endTrajectory().fresh()
                .waitSeconds(0.3)
                .setTangent(-Math.PI/2)
                .lineToY(-67, null, new ProfileAccelConstraint(-70.0, 70.0));

        TrajectoryActionBuilder secondSamplePickupToBar = attachToSecondSample.endTrajectory().fresh()
                .setTangent(Math.PI / 2)
                .splineToSplineHeading(new Pose2d(-0.5, -27, Math.PI / 2), Math.PI / 2);

        TrajectoryActionBuilder bookItToObservationZoneBuilder = secondSamplePickupToBar.endTrajectory().fresh()
                .setTangent(-Math.PI/2)
                .splineTo(new Vector2d(30, -60), 0)
                .setTangent(0)
                .splineToConstantHeading(new Vector2d(60, -60), 0);

        Action moveArmToHighBarAction
                = new MoveArmToPositionAction(arm, HIGH_BAR_VERTICAL_INCHES, HIGH_BAR_HORIZONTAL_INCHES);
        Action clipOnHighBarAction
                = new MoveArmToPositionAction(arm, CLIPPING_VERTICAL_INCHES, HIGH_BAR_HORIZONTAL_INCHES);
        Action moveArmToWallPickupPositionAction
                = new MoveArmToPositionAction(arm, WALL_VERTICAL_INCHES, WALL_HORIZONTAL_INCHES);
        Action pickupFromWallAction
                = new MoveArmToPositionAction(arm, PICKUP_VERTICAL_INCHES, WALL_HORIZONTAL_INCHES);

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        new HomeArmAction(arm),
                        new ParallelAction(
                                startToBarAction.build(),
                                moveArmToHighBarAction
                        ),
                        clipOnHighBarAction,
                        new ParallelAction(
                                moveArmToWallPickupPositionAction,
                                pushFirstTwoSamplesIntoToObservationZoneAction.build()
                        ),
                        attachToFirstSample.build(),
                        pickupFromWallAction,
                        new ParallelAction(
                                moveArmToWallPickupPositionAction,
                                firstSamplePickupToBar.build()
                        ),
                        clipOnHighBarAction,
                        new ParallelAction(
                                moveArmToWallPickupPositionAction,
                                barToSecondSamplePickup.build()
                        ),
                        attachToSecondSample.build(),
                        pickupFromWallAction,
                        new ParallelAction(
                                moveArmToHighBarAction,
                                secondSamplePickupToBar.build()
                        ),
                        clipOnHighBarAction,
                        new ParallelAction(
                                new MoveArmToPositionAction(arm, 0.0, 0.0),
                                bookItToObservationZoneBuilder.build()
                        )
                )
        );
    }
}
