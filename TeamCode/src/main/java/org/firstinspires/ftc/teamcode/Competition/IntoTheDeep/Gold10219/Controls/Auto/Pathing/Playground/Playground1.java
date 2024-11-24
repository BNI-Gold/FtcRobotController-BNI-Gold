package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Playground;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.BotPose.Pinpoint;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.BotPose.PoseHelper;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.BotPose.Vision;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Utils.EasyPoint;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Utils.Offsets;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Utils.Pathing.EasySafePath;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Utils.Pathing.HeadingTypes;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Vars.FieldPoses;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Robots.CompBot.CompBot;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Robots.CompBot.CompBotVars;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.pedroPathing.util.Timer;

@Autonomous(name = "Playground 1", group = "Playground")
public class Playground1 extends OpMode {
    private final CompBot Bot = new CompBot();
    private final CompBotVars vars = new CompBotVars();

    private final Vision vision = new Vision();
    private final Pinpoint pinpoint = new Pinpoint();
    private final PoseHelper pose = new PoseHelper();

    private final FieldPoses poses = new FieldPoses();

    private Pose startPose;

    private Follower follower;

    private Timer pathTimer;

    //This is setup for a theoretical 5-specimen auto
    private Path toSample1, toObservation1, toSample2, toObservation2, toChambers1, toSample3, toObservation3, toChambers2, toObservation4, toChambers3, toObservation5, toChambers4, toObservation6, toChambers5;
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
        toChambers1 = new EasySafePath(startPose, poses.Chambers.Blue,
                new Offsets().addY(vars.Chassis.FRONT_LENGTH).addY(vars.Mechanisms.Grabber.GRABBER_HOOK_POSITION))
                .setHeading(HeadingTypes.CONSTANT, startPose);

        toSample1 = new EasySafePath(startPose, poses.SampleLines.Audience.Blue.B1.Slip, poses.SampleLines.Audience.Blue.B1.Post, poses.SampleLines.Audience.Blue.B1.Sample,
                new Offsets().remY(vars.Chassis.FRONT_LENGTH))
                .setHeading(HeadingTypes.LINEAR, toChambers1, poses.SampleLines.pushApproachAngle, .35);

        toObservation1 = new EasySafePath(toSample1.getLastControlPoint(), poses.SampleLines.Audience.Blue.B1.Pre, poses.Observations.Blue)
                .setHeading(HeadingTypes.CONSTANT, poses.Observations.Blue);

        toSample2 = new EasySafePath(toObservation1.getLastControlPoint(), poses.Observations.Retreats.Blue, poses.SampleLines.Audience.Blue.B2.Pre, poses.SampleLines.Audience.Blue.B2.Post, poses.SampleLines.Audience.Blue.B2.Sample,
                new Offsets().remY(vars.Chassis.FRONT_LENGTH))
                .setHeading(HeadingTypes.CONSTANT, poses.SampleLines.pushApproachAngle);

        toObservation2 = new EasySafePath(toSample2.getLastControlPoint(), poses.Observations.Blue)
                .setHeading(HeadingTypes.CONSTANT, poses.Observations.Blue);

        toChambers2 = new EasySafePath(toObservation2.getLastControlPoint(), poses.Chambers.Blue,
                new Offsets().addY(vars.Chassis.FRONT_LENGTH).addY(vars.Mechanisms.Grabber.GRABBER_HOOK_POSITION))
                .setHeading(HeadingTypes.LINEAR, toObservation2, poses.Chambers.Blue);

        toSample3 = new EasySafePath(toChambers1.getLastControlPoint(), poses.Chambers.Midpoints.Blue, poses.SampleLines.Audience.Blue.B3.Pre, poses.SampleLines.Audience.Blue.B3.Post, poses.SampleLines.Audience.Blue.B3.Sample)
                .setHeading(HeadingTypes.LINEAR, toChambers1, poses.SampleLines.pushApproachAngle);

        toObservation3 = new EasySafePath(toSample3.getLastControlPoint(), poses.Observations.Blue,
                new Offsets().remY(vars.Chassis.FRONT_LENGTH).remY(vars.Mechanisms.Grabber.GRABBER_EXTENDED_POSITION))
                .setHeading(HeadingTypes.LINEAR, toSample3, poses.Observations.Blue);

        toObservation4 = new EasySafePath(toChambers2.getLastControlPoint(), poses.Observations.Blue,
                new Offsets().remY(vars.Chassis.FRONT_LENGTH).remY(vars.Mechanisms.Grabber.GRABBER_EXTENDED_POSITION))
                .setHeading(HeadingTypes.LINEAR, toChambers2, poses.Observations.Blue);

        toChambers3 = new EasySafePath(toObservation4.getLastControlPoint(), poses.Chambers.Midpoints.Blue, poses.Chambers.Blue,
                new Offsets().addY(vars.Chassis.FRONT_LENGTH).addY(vars.Mechanisms.Grabber.GRABBER_HOOK_POSITION))
                .setHeading(HeadingTypes.LINEAR, toObservation4, poses.Chambers.Blue);

        toObservation5 = new EasySafePath(toChambers3.getLastControlPoint(), poses.Observations.Blue,
                new Offsets().remY(vars.Chassis.FRONT_LENGTH).remY(vars.Mechanisms.Grabber.GRABBER_EXTENDED_POSITION))
                .setHeading(HeadingTypes.LINEAR, toChambers3, poses.Observations.Blue);

        toChambers4 = new EasySafePath(toObservation5.getLastControlPoint(), poses.Chambers.Midpoints.Blue, poses.Chambers.Blue,
                new Offsets().addY(vars.Chassis.FRONT_LENGTH).addY(vars.Mechanisms.Grabber.GRABBER_HOOK_POSITION))
                .setHeading(HeadingTypes.LINEAR, toObservation5, poses.Chambers.Blue);

        toObservation6 = new EasySafePath(toChambers4.getLastControlPoint(), poses.Observations.Blue,
                new Offsets().remY(vars.Chassis.FRONT_LENGTH).remY(vars.Mechanisms.Grabber.GRABBER_EXTENDED_POSITION))
                .setHeading(HeadingTypes.LINEAR, toChambers4, poses.Observations.Blue);

        toChambers5 = new EasySafePath(toObservation6.getLastControlPoint(), poses.Chambers.Midpoints.Blue, poses.Chambers.Blue,
                new Offsets().addY(vars.Chassis.FRONT_LENGTH).addY(vars.Mechanisms.Grabber.GRABBER_HOOK_POSITION))
                .setHeading(HeadingTypes.LINEAR, toObservation6, poses.Chambers.Blue);

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 10:
                follower.followPath(toChambers1);
                setPathState(11);
                break;
            case 11:
                if (follower.getCurrentTValue() > 0.1) {
                    ((EasySafePath) toChambers1).setHeading(HeadingTypes.LINEAR, startPose, poses.Chambers.Blue);
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    follower.holdPoint(
                            new EasyPoint(poses.Chambers.Blue,
                                    new Offsets().addY(vars.Chassis.FRONT_LENGTH).addY(vars.Mechanisms.Grabber.GRABBER_HOOK_POSITION)),
                            poses.Chambers.Blue.getHeading()
                    );
                    setPathState(13);
                }
                break;
            case 13:
                if (pathTimer.getElapsedTime() > 500) {
                    setPathState(14);
                }
                break;
            case 14:
                follower.followPath(toSample1);
                setPathState(15);
                break;
            case 15:
                if (!follower.isBusy()) {
                    follower.holdPoint(
                            new EasyPoint(poses.SampleLines.Audience.Blue.B1.Sample),
                            poses.SampleLines.pushApproachAngle
                    );
                    setPathState(16);
                }
                break;
            case 16:
//                if (pathTimer.getElapsedTime() > 500) {
//
//                }
                setPathState(17);
                break;
            case 17:
                follower.followPath(toObservation1);
                setPathState(18);
                break;
            case 18:
                if (!follower.isBusy()) {
                    follower.holdPoint(
                            new EasyPoint(poses.Observations.Blue),
                            poses.Observations.Blue.getHeading()
                    );
                    setPathState(19);
                }
                break;
            case 19:
                if (pathTimer.getElapsedTime() > 500) {
                    setPathState(20);
                }
                break;
            case 20:
                follower.followPath(toSample2);
                setPathState(21);
                break;
            case 21:
                if (!follower.isBusy()) {
                    follower.holdPoint(
                            new EasyPoint(poses.SampleLines.Audience.Blue.B2.Sample),
                            poses.SampleLines.pushApproachAngle
                    );
                    l = follower.getPose();
                    setPathState(22);
                }
                break;
            case 22:
//                if (pathTimer.getElapsedTime() > 500) {
//
//                }
                setPathState(23);
                break;
            case 23:
                follower.followPath(toObservation2);
                setPathState(24);
                break;
            case 24:
                if (!follower.isBusy()) {
                    follower.holdPoint(
                            new EasyPoint(poses.Observations.Blue),
                            poses.Observations.Blue.getHeading()
                    );
                    setPathState(25);
                }
                break;
            case 25:
                if (pathTimer.getElapsedTime() > 500) {
                    setPathState(26);
                }
                break;
            case 26:
                follower.followPath(toChambers2);
                setPathState(27);
                break;
            case 27:
                if (!follower.isBusy()) {
                    follower.holdPoint(
                            new EasyPoint(poses.Chambers.Blue,
                                    new Offsets().addY(vars.Chassis.FRONT_LENGTH).addY(vars.Mechanisms.Grabber.GRABBER_HOOK_POSITION)),
                            poses.Chambers.Blue.getHeading()
                    );
//                    setPathState(28);
                }
                break;
        }
    }

    public void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
        autonomousPathUpdate();
    }
}
