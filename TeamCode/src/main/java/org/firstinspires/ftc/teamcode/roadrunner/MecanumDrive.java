package org.firstinspires.ftc.teamcode.roadrunner;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.lang.Math;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

@Config
public class MecanumDrive {
    // ---------------------------------------------------------------------------------------------
    // Physical Parameters
    // ---------------------------------------------------------------------------------------------

    public static RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
            RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

    private DriveMode driveMode;

    // ---------------------------------------------------------------------------------------------
    // Roadrunner Parameters
    // ---------------------------------------------------------------------------------------------

    // Drive Model Parameters
    public static double INCHES_PER_TICK = 0.0019696;
    public static double LATERAL_INCHES_PER_TICK = INCHES_PER_TICK;
    public static double TRACK_WIDTH_TICKS = 5899.0;

    // Feedforward Parameters (ticks)
    public static double KS = 1.40;
    // Weird behaviour had to turn down originally calculated 0.00257
    public static double KV = 0.000207;
    // Bad deceleration cannot tune higher
    public static double KA = 0.00006;

    // Path Profile Parameters
    public static double MAX_WHEEL_VELOCITY = 80;
    public static double MIN_PROFILE_ACCELERATION = -32.5;
    public static double MAX_PROFILE_ACCELERATION = 70;

    // Turn Profile Parameters (in radians)
    public static double MAX_ANGULAR_VELOCITY_RADIANS = Math.PI; // shared with path
    public static double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND = Math.PI;

    // Path Controller Gain
    public static double AXIAL_GAIN = 16.0;
    public static double LATERAL_GAIN = 8.0;
    public static double HEADING_GAIN = 8.0; // shared with turn

    public static double AXIAL_VELOCITY_GAIN = 0.8;
    public static double LATERAL_VELOCITY_GAIN = 0.8;
    public static double HEADING_VELOCITY_GAIN = 0.8;

    public final MecanumKinematics kinematics = new MecanumKinematics(
            INCHES_PER_TICK * TRACK_WIDTH_TICKS, INCHES_PER_TICK / LATERAL_INCHES_PER_TICK);

    public final TurnConstraints defaultTurnConstraints = new TurnConstraints(
            MAX_ANGULAR_VELOCITY_RADIANS, -MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND, MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND);
    public final VelConstraint defaultVelConstraint =
            new MinVelConstraint(Arrays.asList(
                    kinematics.new WheelVelConstraint(MAX_WHEEL_VELOCITY),
                    new AngularVelConstraint(MAX_ANGULAR_VELOCITY_RADIANS)
            ));
    public final AccelConstraint defaultAccelConstraint =
            new ProfileAccelConstraint(MIN_PROFILE_ACCELERATION, MAX_PROFILE_ACCELERATION);

    public final DcMotorEx frontLeftDriveMotor, backLeftDriveMotor, backRightDriveMotor, frontRightDriveMotor;

    public final VoltageSensor voltageSensor;

    public final LazyImu lazyImu;

    public final Localizer localizer;
    public Pose2d pose;

    public final LinkedList<Pose2d> poseHistory = new LinkedList<>();

    protected MecanumDrive(HardwareMap hardwareMap, Pose2d pose, DriveMode driveMode) {
        this.pose = pose;
        this.driveMode = driveMode;

        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        frontLeftDriveMotor = hardwareMap.get(DcMotorEx.class, "frontLeftDriveMotor");
        backLeftDriveMotor = hardwareMap.get(DcMotorEx.class, "backLeftDriveMotor");
        backRightDriveMotor = hardwareMap.get(DcMotorEx.class, "backRightDriveMotor");
        frontRightDriveMotor = hardwareMap.get(DcMotorEx.class, "frontRightDriveMotor");

        frontLeftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        lazyImu = new LazyImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                logoFacingDirection, usbFacingDirection));

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        localizer = new TwoDeadWheelLocalizer(hardwareMap, lazyImu.get(), INCHES_PER_TICK);
    }

    public void drive(double drive, double strafe, double turn) {
        double thetaRadians = StrictMath.atan2(drive, strafe);

        if (driveMode == DriveMode.FIELD_CENTRIC) {
            thetaRadians -= lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        }

        double power = StrictMath.hypot(strafe, drive);

        double sin_theta = StrictMath.sin(thetaRadians - Math.PI / 4.0);
        double cos_theta = StrictMath.cos(thetaRadians - Math.PI / 4.0);

        double max = Math.max(Math.abs(cos_theta), Math.abs(sin_theta));

        double frontLeftPower  = power * cos_theta / max + turn;
        double frontRightPower = power * sin_theta / max - turn;
        double backLeftPower   = power * sin_theta / max + turn;
        double backRightPower  = power * cos_theta / max - turn;

        double turnMagnitude = Math.abs(turn);

        if ((power + turnMagnitude) > 1.0) {
            frontLeftPower  /= power + turnMagnitude;
            frontRightPower /= power + turnMagnitude;
            backLeftPower   /= power + turnMagnitude;
            backRightPower  /= power + turnMagnitude;
        }

        frontLeftDriveMotor.setPower(frontLeftPower);
        frontRightDriveMotor.setPower(frontRightPower);
        backLeftDriveMotor.setPower(backLeftPower);
        backRightDriveMotor.setPower(backRightPower);
    }

    public void setDrivePowers(PoseVelocity2d powers) {
        MecanumKinematics.WheelVelocities<Time> wheelVels = new MecanumKinematics(1).inverse(
                PoseVelocity2dDual.constant(powers, 1));

        double maxPowerMag = 1;
        for (DualNum<Time> power : wheelVels.all()) {
            maxPowerMag = Math.max(maxPowerMag, power.value());
        }

        frontLeftDriveMotor.setPower(wheelVels.leftFront.get(0) / maxPowerMag);
        backLeftDriveMotor.setPower(wheelVels.leftBack.get(0) / maxPowerMag);
        backRightDriveMotor.setPower(wheelVels.rightBack.get(0) / maxPowerMag);
        frontRightDriveMotor.setPower(wheelVels.rightFront.get(0) / maxPowerMag);
    }

    public final class FollowTrajectoryAction implements Action {
        public final TimeTrajectory timeTrajectory;
        private double beginTs = -1;

        private final double[] xPoints, yPoints;

        public FollowTrajectoryAction(TimeTrajectory t) {
            timeTrajectory = t;

            List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                    0, t.path.length(),
                    Math.max(2, (int) Math.ceil(t.path.length() / 2)));
            xPoints = new double[disps.size()];
            yPoints = new double[disps.size()];
            for (int i = 0; i < disps.size(); i++) {
                Pose2d p = t.path.get(disps.get(i), 1).value();
                xPoints[i] = p.position.x;
                yPoints[i] = p.position.y;
            }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= timeTrajectory.duration) {
                frontLeftDriveMotor.setPower(0);
                backLeftDriveMotor.setPower(0);
                backRightDriveMotor.setPower(0);
                frontRightDriveMotor.setPower(0);

                return false;
            }

            Pose2dDual<Time> txWorldTarget = timeTrajectory.get(t);

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    AXIAL_GAIN, LATERAL_GAIN, HEADING_GAIN,
                    AXIAL_VELOCITY_GAIN, LATERAL_VELOCITY_GAIN, HEADING_VELOCITY_GAIN
            )
                    .compute(txWorldTarget, pose, robotVelRobot);

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();

            final MotorFeedforward feedforward = new MotorFeedforward(KS,
                    KV / INCHES_PER_TICK, KA / INCHES_PER_TICK);
            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;

            frontLeftDriveMotor.setPower(leftFrontPower);
            backLeftDriveMotor.setPower(leftBackPower);
            backRightDriveMotor.setPower(rightBackPower);
            frontRightDriveMotor.setPower(rightFrontPower);

            p.put("x", pose.position.x);
            p.put("y", pose.position.y);
            p.put("heading (deg)", Math.toDegrees(pose.heading.toDouble()));

            Pose2d error = txWorldTarget.value().minusExp(pose);
            p.put("xError", error.position.x);
            p.put("yError", error.position.y);
            p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));

            // only draw when active; only one drive action should be active at a time
            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, pose);

            c.setStroke("#4CAF50FF");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#4CAF507A");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);
        }
    }

    public final class TurnAction implements Action {
        private final TimeTurn turn;

        private double beginTs = -1;

        public TurnAction(TimeTurn turn) {
            this.turn = turn;
        }

        @Override public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= turn.duration) {
                frontLeftDriveMotor.setPower(0);
                backLeftDriveMotor.setPower(0);
                backRightDriveMotor.setPower(0);
                frontRightDriveMotor.setPower(0);

                return false;
            }

            Pose2dDual<Time> txWorldTarget = turn.get(t);

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    AXIAL_GAIN, LATERAL_GAIN, HEADING_GAIN,
                    AXIAL_VELOCITY_GAIN, LATERAL_VELOCITY_GAIN, HEADING_VELOCITY_GAIN
            )
                    .compute(txWorldTarget, pose, robotVelRobot);

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();
            final MotorFeedforward feedforward = new MotorFeedforward(KS,
                    KV / INCHES_PER_TICK, KA / INCHES_PER_TICK);
            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;

            frontLeftDriveMotor.setPower(feedforward.compute(wheelVels.leftFront) / voltage);
            backLeftDriveMotor.setPower(feedforward.compute(wheelVels.leftBack) / voltage);
            backRightDriveMotor.setPower(feedforward.compute(wheelVels.rightBack) / voltage);
            frontRightDriveMotor.setPower(feedforward.compute(wheelVels.rightFront) / voltage);

            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, pose);

            c.setStroke("#7C4DFFFF");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#7C4DFF7A");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }
    }

    public PoseVelocity2d updatePoseEstimate() {
        Twist2dDual<Time> twist = localizer.update();
        pose = pose.plus(twist.value());

        poseHistory.add(pose);
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        return twist.velocity().value();
    }

    private void drawPoseHistory(Canvas c) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];

        int i = 0;
        for (Pose2d t : poseHistory) {
            xPoints[i] = t.position.x;
            yPoints[i] = t.position.y;

            i++;
        }

        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xPoints, yPoints);
    }

    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
                new TrajectoryBuilderParams(
                        1e-6,
                        new ProfileParams(
                                0.25, 0.1, 1e-2
                        )
                ),
                beginPose, 0.0,
                defaultTurnConstraints,
                defaultVelConstraint, defaultAccelConstraint
        );
    }

    public static class Builder {
        private HardwareMap hardwareMap;
        private Pose2d pose;
        private DriveMode driveMode;

        public Builder(@NonNull HardwareMap hardwareMap) {
            this.hardwareMap = hardwareMap;
        }

        public Builder setPose(@NonNull Pose2d pose) {
            this.pose = pose;
            return this;
        }

        public Builder setDriveMode(@NonNull DriveMode mode) {
            this.driveMode = mode;
            return this;
        }

        public MecanumDrive build() {
            if (this.pose == null) {
                pose = new Pose2d(0,0,0);
            }
            if (this.driveMode == null) {
                driveMode = DriveMode.ROBOT_CENTRIC;
            }

            return new MecanumDrive(this.hardwareMap, pose, driveMode);
        }
    }
}
