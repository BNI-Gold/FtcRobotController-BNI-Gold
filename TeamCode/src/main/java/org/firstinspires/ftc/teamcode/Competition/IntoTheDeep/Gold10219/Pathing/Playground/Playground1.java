package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Pathing.Playground;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.BotPose.Pinpoint;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.BotPose.PoseHelper;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.BotPose.Vision;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Pathing.Utils.EasyPath;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Pathing.Utils.SafeInterpolationStartHeading;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Pathing.Vars.FieldPoses;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Robots.CompBot.CompBot;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Robots.CompBot.CompBotVars;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.pedroPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.pedroPathing.util.Timer;

@Autonomous(name = "Playground 1", group = "Playground")
public class Playground1 extends OpMode {
    private CompBot Bot = new CompBot();
    private CompBotVars vars = new CompBotVars();

    private Vision vision = new Vision();
    private Pinpoint pinpoint = new Pinpoint();
    private PoseHelper pose = new PoseHelper();

    private FieldPoses poses = new FieldPoses();

    private Pose startPose;

    private Follower follower;

    private Timer pathTimer;

    private Path toSample1, toObservation1, toSample2, toObservation2, toChambers1, toSample3, toObservation3, toChambers2;
    private Path localizeAfterObservation1;

    private int pathState;

    @Override
    public void init() {
        Bot.initRobot(hardwareMap);

        pinpoint.setOp(this);
        pinpoint.initPinpoint(hardwareMap);

        vision.setOp(this);
        vision.initVision(hardwareMap, pinpoint, false);

        pose.setOp(this);
        pose.setDevices(vision, pinpoint);

        pathTimer = new Timer();

        vision.start();

        pose.updateLLUsage(false);

        pose.updateHeading();
        pose.syncPose();

        pose.updatePose();

        Pose2D currentPose = pose.getPose();
        telemetry.addData("Pose X: ", currentPose.getX(DistanceUnit.INCH));
        telemetry.addData("Pose Y: ", currentPose.getY(DistanceUnit.INCH));
        telemetry.addData("Pose H: ", currentPose.getHeading(AngleUnit.DEGREES));

        startPose = new Pose(
                currentPose.getX(DistanceUnit.INCH),
                currentPose.getY(DistanceUnit.INCH),
                currentPose.getHeading(AngleUnit.RADIANS)
        );

        telemetry.addData("Start Pose: ", startPose);
        telemetry.update();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
    }

    public void start() {
        buildPaths();
        setPathState(10);
    }

    public void loop() {
        pose.updatePose();
        follower.update();
        autonomousPathUpdate();
        tel();
    }

    Pose l = null;

    public void tel() {
        Pose2D current = pose.getPose();
        telemetry.addData("PX: ", current.getX(DistanceUnit.INCH));
        telemetry.addData("PY: ", current.getY(DistanceUnit.INCH));
        telemetry.addData("PO: ", current.getHeading(AngleUnit.DEGREES));
        telemetry.addLine();
        telemetry.addData("pathState: ", pathState);
        if (l != null) {
            telemetry.addLine();
            telemetry.addData("LX: ", l.getX());
            telemetry.addData("LY: ", l.getY());
            telemetry.addData("LH: ", l.getHeading());
        }
        telemetry.update();
    }

    public void buildPaths() {
        toSample1 = new EasyPath(startPose, poses.SampleLines.Audience.Blue1);
        toSample1.setConstantHeadingInterpolation(startPose.getHeading());
        toSample1.setPathEndTimeoutConstraint(3);

        toObservation1 = new EasyPath(toSample1.getLastControlPoint(), poses.Observations.Blue, new double[]{}, new double[]{-vars.Chassis.FRONT_LENGTH - vars.Mechanisms.Grabber.GRABBER_EXTENDED_POSITION});
        toObservation1.setLinearHeadingInterpolation(new SafeInterpolationStartHeading(toSample1.getEndTangent().getTheta(), poses.Observations.Blue).getValue(), poses.Observations.Blue.getHeading(), .8);
        toObservation1.setPathEndTimeoutConstraint(3);

        localizeAfterObservation1 = new EasyPath(toSample1.getLastControlPoint(), poses.Recalibration.Single.A11);
        localizeAfterObservation1.setLinearHeadingInterpolation(new SafeInterpolationStartHeading(toSample1.getEndTangent().getTheta(), poses.Recalibration.Single.A11).getValue(), poses.Recalibration.Single.A11.getHeading());
        localizeAfterObservation1.setPathEndTimeoutConstraint(3);

        toSample2 = new EasyPath(toObservation1.getLastControlPoint(), poses.SampleLines.Audience.Blue1);
        toSample2.setLinearHeadingInterpolation(new SafeInterpolationStartHeading(toObservation1.getEndTangent().getTheta(), poses.SampleLines.Audience.Blue1).getValue(), poses.SampleLines.Audience.Blue1.getHeading(), .8);
        toSample2.setPathEndTimeoutConstraint(3);

        toObservation2 = new EasyPath(toSample2.getLastControlPoint(), poses.Observations.Blue, new double[]{}, new double[]{-vars.Chassis.FRONT_LENGTH - vars.Mechanisms.Grabber.GRABBER_EXTENDED_POSITION});
        toObservation2.setLinearHeadingInterpolation(new SafeInterpolationStartHeading(toSample2.getEndTangent().getTheta(), poses.Observations.Blue).getValue(), poses.Observations.Blue.getHeading(), .8);
        toObservation2.setPathEndTimeoutConstraint(3);

        toChambers1 = new EasyPath(toObservation2.getLastControlPoint(), poses.Chambers.Midpoints.Blue, poses.Chambers.Blue, new double[]{}, new double[]{vars.Chassis.FRONT_LENGTH + vars.Mechanisms.Grabber.GRABBER_HOOK_POSITION});
        toChambers1.setLinearHeadingInterpolation(new SafeInterpolationStartHeading(toObservation2.getEndTangent().getTheta(), poses.Chambers.Blue).getValue(), poses.Chambers.Blue.getHeading(), .8);
        toChambers1.setPathEndTimeoutConstraint(3);

        toSample3 = new EasyPath(toChambers1.getLastControlPoint(), new Point(48, 144-48), poses.SampleLines.Audience.Blue1);
        toSample3.setLinearHeadingInterpolation(new SafeInterpolationStartHeading(toChambers1.getEndTangent().getTheta(), poses.SampleLines.Audience.Blue1).getValue(), poses.SampleLines.Audience.Blue1.getHeading(), .8);
        toSample3.setPathEndTimeoutConstraint(3);

        toObservation3 = new EasyPath(toSample3.getLastControlPoint(), poses.Observations.Blue);
        toObservation3.setLinearHeadingInterpolation(new SafeInterpolationStartHeading(toSample3.getEndTangent().getTheta(), poses.Observations.Blue).getValue(), poses.Observations.Blue.getHeading(), 0.8);
        toObservation3.setPathEndTimeoutConstraint(3);

        toChambers2 = new EasyPath(toObservation2.getLastControlPoint(), poses.Chambers.Midpoints.Blue, poses.Chambers.Blue);
        toChambers2.setLinearHeadingInterpolation(new SafeInterpolationStartHeading(toObservation3.getEndTangent().getTheta(), poses.Chambers.Blue).getValue(), poses.Chambers.Blue.getHeading(), 0.8);
        toChambers2.setPathEndTimeoutConstraint(3);

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 10:
                follower.followPath(toSample1);
                setPathState(11);
                break;
            case 11:
                if (follower.getCurrentTValue() > 0.1) {
                    toSample1.setLinearHeadingInterpolation(new SafeInterpolationStartHeading(startPose, poses.SampleLines.Audience.Blue1).getValue(), poses.SampleLines.Audience.Blue1.getHeading());
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    follower.holdPoint(new BezierPoint(toSample1.getLastControlPoint()), toSample1.getEndTangent().getTheta());
                    setPathState(13);
                }
                break;
            case 13:
                if (pathTimer.getElapsedTime() > 1000) {
                    setPathState(14);
                }
                break;
            case 14:
                follower.followPath(toObservation1);
                setPathState(15);
                break;
            case 15:
                if (!follower.isBusy()) {
                    follower.holdPoint(new BezierPoint(generateOffsetPoint(poses.Observations.Blue, new double[]{}, new double[]{-vars.Chassis.FRONT_LENGTH - vars.Mechanisms.Grabber.GRABBER_EXTENDED_POSITION})), poses.Observations.Blue.getHeading());
                    setPathState(16);
                }
                break;
            case 16:
                if (pathTimer.getElapsedTime() > 1000) {
                    setPathState(17);
                }
                break;
            case 17:
//                follower.followPath(localizeAfterObservation1);
                follower.followPath(toSample2);
                setPathState(18);
                break;
            case 18:
                if (!follower.isBusy()) {
                    follower.holdPoint(new BezierPoint(generateOffsetPoint(poses.SampleLines.Audience.Blue1, new double[]{}, new double[]{})), poses.SampleLines.Audience.Blue1.getHeading());
//                    follower.holdPoint(new BezierPoint(new Point(poses.Recalibration.Single.A11)), poses.Recalibration.Single.A11.getHeading());
                    l = follower.getPose();
                    setPathState(19);

                }
                break;
            case 19:
                if (pathTimer.getElapsedTime() > 1000) {
                    setPathState(20);
                }
                break;
            case 20:
                follower.followPath(toObservation2);
                setPathState(21);
                break;
            case 21:
                if (!follower.isBusy()) {
                    follower.holdPoint(new BezierPoint(generateOffsetPoint(poses.Observations.Blue, new double[]{}, new double[]{-vars.Chassis.FRONT_LENGTH - vars.Mechanisms.Grabber.GRABBER_EXTENDED_POSITION})), poses.Observations.Blue.getHeading());
                    l = follower.getPose();
                    setPathState(22);
                }
                break;
            case 22:
                if (pathTimer.getElapsedTime() > 1000) {
                    setPathState(23);
                }
                break;
            case 23:
                follower.followPath(toChambers1);
                setPathState(24);
                break;
            case 24:
                if (!follower.isBusy()) {
                    follower.holdPoint(new BezierPoint(generateOffsetPoint(poses.Chambers.Blue, new double[]{}, new double[]{vars.Chassis.FRONT_LENGTH + vars.Mechanisms.Grabber.GRABBER_HOOK_POSITION})), poses.Chambers.Blue.getHeading());
                    setPathState(25);
                }
                break;
            case 25:
                if (pathTimer.getElapsedTime() > 1000) {
                    setPathState(26);
                }
                break;
            case 26:
                follower.followPath(toObservation3);
                setPathState(27);
                break;
            case 27:
                if (!follower.isBusy()) {
                    follower.holdPoint(new BezierPoint(generateOffsetPoint(poses.Observations.Blue, new double[]{}, new double[]{-vars.Chassis.FRONT_LENGTH - vars.Mechanisms.Grabber.GRABBER_EXTENDED_POSITION})), poses.Observations.Blue.getHeading());
                    setPathState(28);
                }
                break;
            case 28:
                if (pathTimer.getElapsedTime() > 1000) {
                    setPathState(29);
                }
                break;
            case 29:
                follower.followPath(toChambers2);
                setPathState(30);
                break;
            case 30:
                if (!follower.isBusy()) {
                    follower.holdPoint(new BezierPoint(generateOffsetPoint(poses.Chambers.Blue, new double[]{}, new double[]{vars.Chassis.FRONT_LENGTH + vars.Mechanisms.Grabber.GRABBER_HOOK_POSITION})), poses.Chambers.Blue.getHeading());
                }
        }
    }

    public void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
        autonomousPathUpdate();
    }

    public static Point generateOffsetPoint(Point finalPoint, double[] xOffsets, double[] yOffsets) {
        double xOffsetSum = 0;
        double yOffsetSum = 0;

        for (int i = 0; i < xOffsets.length; i++) {
            xOffsetSum += xOffsets[i];
        }

        for (int i = 0; i < yOffsets.length; i++) {
            yOffsetSum += yOffsets[i];
        }

        return new Point(finalPoint.getX() + xOffsetSum, finalPoint.getY() + yOffsetSum);
    }

    public static Point generateOffsetPoint(Pose finalPose, double[] xOffsets, double[] yOffsets) {
        double xOffsetSum = 0;
        double yOffsetSum = 0;

        for (int i = 0; i < xOffsets.length; i++) {
            xOffsetSum += xOffsets[i];
        }

        for (int i = 0; i < yOffsets.length; i++) {
            yOffsetSum += yOffsets[i];
        }

        return new Point(finalPose.getX() + xOffsetSum, finalPose.getY() + yOffsetSum);
    }
}
