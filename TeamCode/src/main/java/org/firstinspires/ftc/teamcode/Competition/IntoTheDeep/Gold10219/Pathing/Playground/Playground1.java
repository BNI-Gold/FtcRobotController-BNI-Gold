package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Pathing.Playground;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.BotPose.Pinpoint;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.BotPose.PoseHelper;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.BotPose.Vision;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Pathing.Utils.EasyPoint;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Pathing.Utils.Offsets;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Pathing.Utils.Pathing.EasySafePath;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Pathing.Utils.Pathing.HeadingPath;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Pathing.Utils.Pathing.HeadingTypes;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Pathing.Vars.FieldPoses;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Robots.CompBot.CompBot;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Robots.CompBot.CompBotVars;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.pedroPathing.localization.Pose;
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
        toSample1 = new EasySafePath(startPose, poses.SampleLines.Audience.Blue1)
                .setHeading(HeadingTypes.CONSTANT, startPose);

        toObservation1 = new EasySafePath(toSample1.getLastControlPoint(), poses.Observations.Blue,
                new Offsets().remY(vars.Chassis.FRONT_LENGTH).remY(vars.Mechanisms.Grabber.GRABBER_EXTENDED_POSITION))
                .setHeading(HeadingTypes.LINEAR, toSample1, poses.Observations.Blue);

        toSample2 = new EasySafePath(toObservation1.getLastControlPoint(), poses.SampleLines.Audience.Blue1)
                .setHeading(HeadingTypes.LINEAR, toObservation1, poses.SampleLines.Audience.Blue1);

        toObservation2 = new EasySafePath(toSample2.getLastControlPoint(), poses.Observations.Blue,
                new Offsets().remY(vars.Chassis.FRONT_LENGTH).remY(vars.Mechanisms.Grabber.GRABBER_EXTENDED_POSITION))
                .setHeading(HeadingTypes.LINEAR, toSample2, poses.Observations.Blue);

        toChambers1 = new EasySafePath(toObservation2.getLastControlPoint(), poses.Chambers.Midpoints.Blue, poses.Chambers.Blue,
                new Offsets().remY(vars.Chassis.FRONT_LENGTH).remY(vars.Mechanisms.Grabber.GRABBER_EXTENDED_POSITION))
                .setHeading(HeadingTypes.LINEAR, toObservation2, poses.Chambers.Blue);

        toSample3 = new EasySafePath(toChambers1.getLastControlPoint(), new Point(48, 144 - 48), poses.SampleLines.Audience.Blue1)
                .setHeading(HeadingTypes.LINEAR, toChambers1, poses.SampleLines.Audience.Blue1);

        toObservation3 = new EasySafePath(toSample3.getLastControlPoint(), poses.Observations.Blue)
                .setHeading(HeadingTypes.LINEAR, toSample3, poses.Observations.Blue);

        toChambers2 = new EasySafePath(toObservation2.getLastControlPoint(), poses.Chambers.Midpoints.Blue, poses.Chambers.Blue)
                .setHeading(HeadingTypes.LINEAR, toObservation3, poses.Chambers.Blue);

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 10:
                follower.followPath(toSample1);
                setPathState(11);
                break;
            case 11:
                if (follower.getCurrentTValue() > 0.1) {
                    //TODO: if initial heading doesn't work as expected, this is why.
//                    toSample1.setLinearHeadingInterpolation(new SafeInterpolationStartHeading(startPose, poses.SampleLines.Audience.Blue1).getValue(), poses.SampleLines.Audience.Blue1.getHeading());
                    ((HeadingPath) toSample1).setHeading(HeadingTypes.LINEAR, startPose, poses.SampleLines.Audience.Blue1);
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    follower.holdPoint(new EasyPoint(poses.SampleLines.Audience.Blue1), poses.SampleLines.Audience.Blue1.getHeading());
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
                    follower.holdPoint(
                            new EasyPoint(poses.Observations.Blue,
                                    new Offsets().remY(-vars.Chassis.FRONT_LENGTH).remY(vars.Mechanisms.Grabber.GRABBER_EXTENDED_POSITION)),
                            poses.Observations.Blue.getHeading());
                    setPathState(16);
                }
                break;
            case 16:
                if (pathTimer.getElapsedTime() > 1000) {
                    setPathState(17);
                }
                break;
            case 17:
                follower.followPath(toSample2);
                setPathState(18);
                break;
            case 18:
                if (!follower.isBusy()) {
                    follower.holdPoint(new EasyPoint(poses.SampleLines.Audience.Blue1), poses.SampleLines.Audience.Blue1.getHeading());
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
                    follower.holdPoint(
                            new EasyPoint(poses.Observations.Blue,
                                    new Offsets().remY(-vars.Chassis.FRONT_LENGTH).remY(vars.Mechanisms.Grabber.GRABBER_EXTENDED_POSITION)),
                            poses.Observations.Blue.getHeading());
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
                    follower.holdPoint(
                            new EasyPoint(poses.Chambers.Blue,
                                    new Offsets().addY(vars.Chassis.FRONT_LENGTH).addY(vars.Mechanisms.Grabber.GRABBER_HOOK_POSITION)),
                            poses.Chambers.Blue.getHeading());
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
                    follower.holdPoint(
                            new EasyPoint(poses.Observations.Blue,
                                    new Offsets().remY(vars.Chassis.FRONT_LENGTH).remY(vars.Mechanisms.Grabber.GRABBER_EXTENDED_POSITION)),
                            poses.Observations.Blue.getHeading());
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
                    follower.holdPoint(
                            new EasyPoint(poses.Chambers.Blue,
                                    new Offsets().addY(vars.Chassis.FRONT_LENGTH).addY(vars.Mechanisms.Grabber.GRABBER_HOOK_POSITION)),
                            poses.Chambers.Blue.getHeading());
                }
        }
    }

    public void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
        autonomousPathUpdate();
    }
}
