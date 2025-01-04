package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue;

import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.approachGrabSpecimen1;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.approachGrabSpecimen1Timeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.approachGrabSpecimen2;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.approachGrabSpecimen2Timeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.chambers1Heading;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.chambers1LowerArm;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.chambers1LowerArmTimeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.chambers1RaiseArm;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.chambers1ReleaseAndBack;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.chambers1ReleaseAndBackTimeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.chambers1Timeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.chambers2Heading;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.chambers2LowerArm;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.chambers2LowerArmTimeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.chambers2RaiseArm;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.chambers2ReleaseAndBack;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.chambers2ReleaseAndBackTimeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.chambers2Timeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.chambers3Heading;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.chambers3LowerArm;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.chambers3LowerArmTimeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.chambers3RaiseArm;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.chambers3ReleaseAndBack;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.chambers3ReleaseAndBackTimeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.chambers3Timeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.grabSpecimen1;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.grabSpecimen1Timeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.grabSpecimen2;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.grabSpecimen2Timeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.grabberOut1;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.grabberOut1Timeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.grabberOut2;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.grabberOut2Timeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.holdChambers1;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.holdChambers2;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.holdChambers3;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.holdObservation1;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.holdObservation1Timeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.holdObservation2;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.holdObservation2Timeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.holdSample1;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.holdSample2;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.liftSpecimen1;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.liftSpecimen1Timeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.liftSpecimen2;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.liftSpecimen2Timeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.observationRetreat1;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.observationRetreat1Timeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.observationRetreat2;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.observationRetreat2Timeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.observationRetreatFromPush1;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.observationRetreatFromPush1Timeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.observationRetreatFromPush2;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.observationRetreatFromPush2Timeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.toChambers1;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.toChambers2;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.toChambers3;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.toObservation1;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.toObservation2;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.toSample1;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_2_PathStates.toSample2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.BotPose.Pinpoint;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Vars.FieldPoses;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Mechanisms.Grabber.Grabber;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Mechanisms.PrimaryArm.PrimaryArm;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Pathing.Utils.EasyPoint;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Pathing.Utils.O;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Pathing.Utils.Offsets;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Pathing.Utils.Paths.EasySafePath;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Pathing.Utils.Paths.HeadingTypes;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Robots.CompBot.CompBot;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Robots.CompBot.CompBotVars;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.pedroPathing.util.Timer;

import java.util.HashMap;
import java.util.Map;

@Autonomous(name = "A - Blue 1 + 2", group = "Auto - Blue")
public class Blue_1_2 extends OpMode {
    private final CompBot Bot = new CompBot();
    private final CompBotVars vars = new CompBotVars();

    private final Pinpoint pinpoint = new Pinpoint();
    private final FieldPoses poses = new FieldPoses();

    Grabber grabber = new Grabber();

    PrimaryArm arm = new PrimaryArm();

    private Pose startPose;

    private Follower follower;

    private Timer pathTimer;

    private final Map<Blue_1_2_PathStates, Path> paths = new HashMap<>();
    private Blue_1_2_PathStates pathState;

    double pushSampleOffset = 2;

    double targetXOffset = -20.5;
    double majorCorrectionFactor = 0.25;
    double minorCorrectionFactor = 0.6;
    double majorMinorBreakpoint = 5;
    double permissibleOffset = .75;

    @Override
    public void init() {
        Bot.initRobot(hardwareMap);

        pinpoint.setOp(this);
        pinpoint.initPinpoint(hardwareMap);

        pathTimer = new Timer();

        startPose = new Pose(
                48,
                136,
                Math.toRadians(180)
        );

        telemetry.addData("Start Pose: ", startPose);
        telemetry.update();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        arm.initPrimaryArm(hardwareMap, Bot.LinearOp);
        grabber.initGrabber(hardwareMap);

        grabber.grab();
        grabber.headStraight();
        grabber.setGrabberState(Grabber.grabberStates.TUCK);
        arm.setRetract();
    }

    public void start() {
        grabber.grab();
        grabber.headStraight();
        grabber.setGrabberState(Grabber.grabberStates.TUCK);
        arm.setRetract();
        buildPaths();
        setPathState(toChambers1);
    }

    public void loop() {
        follower.update();
        grabber.tiltStateCheck();
        arm.rotationChecker();
        autonomousPathUpdate();
        tel();
    }

    Pose l = null;

    public void tel() {
        telemetry.addData("pathState: ", pathState);
        if (l != null) {
            telemetry.addLine();
            telemetry.addData("LX: ", l.getX());
            telemetry.addData("LY: ", l.getY());
            telemetry.addData("LH: ", l.getHeading());
        }
        telemetry.addLine();
        telemetry.addData("Imu Status: ", grabber.imu.getSystemStatus());
        telemetry.update();
    }

    private Path getPath(Blue_1_2_PathStates state) {
        return O.req(paths.get(state));
    }

    public void buildPaths() {
        paths.put(toChambers1,
                new EasySafePath(startPose, poses.Chambers.Blue,
                        new Offsets().addY(vars.Chassis.FRONT_LENGTH).addY(vars.Mechanisms.Grabber.AtChambers.OUT).addX(vars.sample))
                        .setHeading(HeadingTypes.CONSTANT, startPose));

        paths.put(toSample1,
                new EasySafePath(getPath(toChambers1).getLastControlPoint(), poses.SampleLines.Blue.B1.Slip, poses.SampleLines.Blue.B1.Post, poses.SampleLines.Blue.B1.Sample,
                        new Offsets().remY(vars.Chassis.FRONT_LENGTH))
                        .setHeading(HeadingTypes.LINEAR, getPath(toChambers1), poses.SampleLines.Blue.pushApproachAngle, .35));

        paths.put(toObservation1,
                new EasySafePath(getPath(toSample1).getLastControlPoint(), poses.SampleLines.Blue.B1.Pre, poses.Observations.Grabs.Blue,
                        new Offsets().remY(vars.Mechanisms.Grabber.AtObservation.OUT).addY(pushSampleOffset).remX(pushSampleOffset))
                        .setHeading(HeadingTypes.CONSTANT, poses.Observations.Grabs.Blue));

        paths.put(toChambers2,
                new EasySafePath(getPath(toObservation1).getLastControlPoint(), poses.Observations.Retreats.Blue, poses.Chambers.Blue,
                        new Offsets().addY(vars.Chassis.FRONT_LENGTH).addY(vars.Mechanisms.Grabber.AtChambers.OUT))
                        .setHeading(HeadingTypes.CONSTANT, poses.Observations.Grabs.Blue));

        paths.put(toSample2,
                new EasySafePath(getPath(toChambers2).getLastControlPoint(), poses.SampleLines.Blue.B2.Slip, poses.SampleLines.Blue.B2.Post, poses.SampleLines.Blue.B2.Sample,
                        new Offsets().remY(vars.Chassis.FRONT_LENGTH))
                        .setHeading(HeadingTypes.LINEAR, getPath(toChambers2), poses.SampleLines.Blue.pushApproachAngle, .35));

        paths.put(toObservation2,
                new EasySafePath(getPath(toSample2).getLastControlPoint(), poses.SampleLines.Blue.B2.Pre, poses.Observations.Grabs.Blue,
                        new Offsets().remY(vars.Mechanisms.Grabber.AtObservation.OUT).addY(pushSampleOffset).remX(pushSampleOffset))
                        .setHeading(HeadingTypes.CONSTANT, poses.Observations.Grabs.Blue));

        paths.put(toChambers3,
                new EasySafePath(getPath(toObservation2).getLastControlPoint(), poses.Observations.Retreats.Blue, poses.Chambers.Blue,
                        new Offsets().addY(vars.Chassis.FRONT_LENGTH).addY(vars.Mechanisms.Grabber.AtChambers.OUT).remX(vars.sample))
                        .setHeading(HeadingTypes.CONSTANT, poses.Observations.Grabs.Blue));
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case toChambers1:
                follower.followPath(getPath(toChambers1));
                grabber.setGrabberState(Grabber.grabberStates.TUCK);
                setPathState(chambers1Heading);
                break;
            case chambers1Heading:
                if (follower.getCurrentTValue() > 0.1) {
                    ((EasySafePath) getPath(toChambers1)).setHeading(HeadingTypes.LINEAR, startPose, poses.Chambers.Blue);
                    setPathState(
                            chambers1RaiseArm);
                }
                break;
            case chambers1RaiseArm:
                arm.setRotation(PrimaryArm.rotationStates.UP, 6, true);
                grabber.setGrabberState(Grabber.grabberStates.OUT);
                setPathState(holdChambers1);
                break;
            case holdChambers1:
                if (!follower.isBusy() && arm.isStopped() && grabber.isSettled()) {
                    follower.holdPoint(
                            new EasyPoint(poses.Chambers.Blue,
                                    new Offsets().addY(vars.Chassis.FRONT_LENGTH).addY(vars.Mechanisms.Grabber.AtChambers.OUT).addX(vars.sample)),
                            poses.Chambers.Blue.getHeading()
                    );
                    setPathState(chambers1Timeout);
                }
                break;
            case chambers1Timeout:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 500 && grabber.isSettled()) {
                    setPathState(chambers1LowerArm);
                }
                break;

            case chambers1LowerArm:
                grabber.setGrabberState(Grabber.grabberStates.HOOK);
                if (pathTimer.getElapsedTime() > 500 && grabber.isSettled()) {
                    arm.setRotation(PrimaryArm.rotationStates.DOWN, 1, false);
                    setPathState(chambers1LowerArmTimeout);
                }
                break;
            case chambers1LowerArmTimeout:
                if (pathTimer.getElapsedTime() > 250 && arm.isStopped()) {
                    setPathState(chambers1ReleaseAndBack);
                }
                break;
            case chambers1ReleaseAndBack:
                grabber.release();
                grabber.setGrabberState(Grabber.grabberStates.OUT);
                follower.holdPoint(
                        new EasyPoint(poses.Chambers.Retreats.Blue,
                                new Offsets().addY(vars.Chassis.FRONT_LENGTH).addY(vars.Mechanisms.Grabber.AtChambers.OUT)),
                        poses.Chambers.Retreats.Blue.getHeading()
                );
                setPathState(chambers1ReleaseAndBackTimeout);
                break;
            case chambers1ReleaseAndBackTimeout:
                if (pathTimer.getElapsedTime() > 500) {
                    setPathState(toSample1);
                }
                break;
            case toSample1:
                follower.followPath(getPath(toSample1));
                setPathState(holdSample1);
                break;
            case holdSample1:
                if (!follower.isBusy()) {
                    grabber.setGrabberState(Grabber.grabberStates.DOWN);
                    follower.holdPoint(
                            new EasyPoint(poses.SampleLines.Blue.B1.Sample),
                            poses.SampleLines.Blue.pushApproachAngle
                    );
                    setPathState(toObservation1);
                }
                break;
            case toObservation1:
                follower.followPath(getPath(toObservation1));
                setPathState(holdObservation1);
                break;
            case holdObservation1:
                if (!follower.isBusy()) {
                    arm.setRotation(PrimaryArm.rotationStates.DOWN, 4, false);
                    follower.holdPoint(
                            new EasyPoint(poses.Observations.Grabs.Blue,
                                    new Offsets().remY(vars.Mechanisms.Grabber.AtObservation.OUT).addY(pushSampleOffset).remX(pushSampleOffset)),
                            poses.Observations.Grabs.Blue.getHeading());

                    setPathState(holdObservation1Timeout);
                }
                break;
            case holdObservation1Timeout:
                if (pathTimer.getElapsedTime() > 500 && !follower.isBusy()) {
                    setPathState(observationRetreatFromPush1);
                }
                break;
            case observationRetreatFromPush1:
                follower.holdPoint(
                        new EasyPoint(poses.Observations.Approaches.Blue),
                        poses.Observations.Approaches.Blue.getHeading());

                setPathState(observationRetreatFromPush1Timeout);
                break;
            case observationRetreatFromPush1Timeout:
                if (pathTimer.getElapsedTime() > 500 & !follower.isBusy()) {
                    setPathState(grabberOut1);
                }
                break;
            case grabberOut1:
                grabber.setGrabberState(Grabber.grabberStates.OUT);
                if (grabber.isSettled()) {
                    setPathState(grabberOut1Timeout);
                }
                break;
            case grabberOut1Timeout:
                if (pathTimer.getElapsedTime() > 250 && grabber.isSettled()) {
                    setPathState(approachGrabSpecimen1);
                }
                break;
            case approachGrabSpecimen1:
                follower.holdPoint(
                        new EasyPoint(poses.Observations.Grabs.Blue,
                                new Offsets().remY(vars.Mechanisms.Grabber.AtObservation.OUT)),
                        poses.Observations.Grabs.Blue.getHeading());
                setPathState(approachGrabSpecimen1Timeout);
                break;
            case approachGrabSpecimen1Timeout:
                if (pathTimer.getElapsedTime() > 750 && !follower.isBusy())
                    setPathState(grabSpecimen1);
                break;
            case grabSpecimen1:
                grabber.grab();
                setPathState(grabSpecimen1Timeout);
                break;
            case grabSpecimen1Timeout:
                if (pathTimer.getElapsedTime() > 500) {
                    setPathState(liftSpecimen1);
                }
                break;
            case liftSpecimen1:
                arm.setRotation(PrimaryArm.rotationStates.UP, 1, true);
                setPathState(liftSpecimen1Timeout);
                break;
            case liftSpecimen1Timeout:
                if (pathTimer.getElapsedTime() > 250 && arm.isStopped()) {
                    setPathState(observationRetreat1);
                }
                break;
            case observationRetreat1:
                follower.holdPoint(
                        new EasyPoint(poses.Observations.Retreats.Blue),
                        poses.Observations.Retreats.Blue.getHeading());
                setPathState(observationRetreat1Timeout);
                break;
            case observationRetreat1Timeout:
                if (pathTimer.getElapsedTime() > 500 && !follower.isBusy()) {
                    setPathState(toChambers2);
                }
                break;
            case toChambers2:
                follower.followPath(getPath(toChambers2));
                setPathState(chambers2RaiseArm);
                break;
            case chambers2RaiseArm:
                arm.setRotation(PrimaryArm.rotationStates.UP, 4, true);
                grabber.setGrabberState(Grabber.grabberStates.OUT);
                setPathState(chambers2Heading);
                break;
            case chambers2Heading:
                if (follower.getCurrentTValue() > 0.3) {
                    ((EasySafePath) getPath(toChambers2)).setHeading(HeadingTypes.LINEAR, poses.Observations.Grabs.Blue, poses.Chambers.Blue);
                    setPathState(holdChambers2);
                }
                break;
            case holdChambers2:
                if (!follower.isBusy() && arm.isStopped()) {
                    follower.holdPoint(
                            new EasyPoint(poses.Chambers.Blue,
                                    new Offsets().addY(vars.Chassis.FRONT_LENGTH).addY(vars.Mechanisms.Grabber.AtChambers.OUT).remY(.75)),
                            poses.Chambers.Blue.getHeading()
                    );
                    setPathState(chambers2Timeout);
                }
                break;
            case chambers2Timeout:
                if (pathTimer.getElapsedTime() > 250) {
                    setPathState(chambers2LowerArm);
                }
                break;

            case chambers2LowerArm:
                grabber.setGrabberState(Grabber.grabberStates.HOOK);
                if (pathTimer.getElapsedTime() > 500 && grabber.isSettled()) {
                    arm.setRotation(PrimaryArm.rotationStates.DOWN, 1, false);
                    setPathState(chambers2LowerArmTimeout);
                }
                break;
            case chambers2LowerArmTimeout:
                if (pathTimer.getElapsedTime() > 200 && arm.isStopped()) {
                    setPathState(chambers2ReleaseAndBack);
                }
                break;
            case chambers2ReleaseAndBack:
                grabber.release();
                grabber.setGrabberState(Grabber.grabberStates.OUT);
                follower.holdPoint(
                        new EasyPoint(poses.Chambers.Retreats.Blue,
                                new Offsets().addY(vars.Chassis.FRONT_LENGTH).addY(vars.Mechanisms.Grabber.AtChambers.OUT)),
                        poses.Chambers.Retreats.Blue.getHeading()
                );
                setPathState(chambers2ReleaseAndBackTimeout);
                break;
            case chambers2ReleaseAndBackTimeout:
                if (pathTimer.getElapsedTime() > 500) {
                    setPathState(toSample2);
                }
                break;
            case toSample2:
                follower.followPath(getPath(toSample2));
                setPathState(holdSample2);
                break;
            case holdSample2:
                if (!follower.isBusy()) {
                    grabber.setGrabberState(Grabber.grabberStates.DOWN);
                    follower.holdPoint(
                            new EasyPoint(poses.SampleLines.Blue.B2.Sample),
                            poses.SampleLines.Blue.pushApproachAngle
                    );
                    setPathState(toObservation2);
                }
                break;
            case toObservation2:
                follower.followPath(getPath(toObservation2));
                setPathState(holdObservation2);
                break;
            case holdObservation2:
                if (!follower.isBusy()) {
                    arm.setRotation(PrimaryArm.rotationStates.DOWN, 4, false);
                    follower.holdPoint(
                            new EasyPoint(poses.Observations.Grabs.Blue,
                                    new Offsets().remY(vars.Mechanisms.Grabber.AtObservation.OUT).addY(pushSampleOffset).remX(pushSampleOffset)),
                            poses.Observations.Grabs.Blue.getHeading());

                    setPathState(holdObservation2Timeout);
                }
                break;
            case holdObservation2Timeout:
                if (pathTimer.getElapsedTime() > 500 && !follower.isBusy()) {
                    setPathState(observationRetreatFromPush2);
                }
                break;
            case observationRetreatFromPush2:
                follower.holdPoint(
                        new EasyPoint(poses.Observations.Approaches.Blue),
                        poses.Observations.Approaches.Blue.getHeading());

                setPathState(observationRetreatFromPush2Timeout);
                break;
            case observationRetreatFromPush2Timeout:
                if (pathTimer.getElapsedTime() > 500 & !follower.isBusy()) {
                    setPathState(grabberOut2);
                }
                break;
            case grabberOut2:
                grabber.setGrabberState(Grabber.grabberStates.OUT);
                if (grabber.isSettled()) {
                    setPathState(grabberOut2Timeout);
                }
                break;
            case grabberOut2Timeout:
                if (pathTimer.getElapsedTime() > 250 && grabber.isSettled()) {
                    setPathState(approachGrabSpecimen2);
                }
                break;
            case approachGrabSpecimen2:
                follower.holdPoint(
                        new EasyPoint(poses.Observations.Grabs.Blue,
                                new Offsets().remY(vars.Mechanisms.Grabber.AtObservation.OUT)),
                        poses.Observations.Grabs.Blue.getHeading());
                setPathState(approachGrabSpecimen2Timeout);
                break;
            case approachGrabSpecimen2Timeout:
                if (pathTimer.getElapsedTime() > 750 && !follower.isBusy())
                    setPathState(grabSpecimen2);
                break;
            case grabSpecimen2:
                grabber.grab();
                setPathState(grabSpecimen2Timeout);
                break;
            case grabSpecimen2Timeout:
                if (pathTimer.getElapsedTime() > 500) {
                    setPathState(liftSpecimen2);
                }
                break;
            case liftSpecimen2:
                arm.setRotation(PrimaryArm.rotationStates.UP, 1, true);
                setPathState(liftSpecimen2Timeout);
                break;
            case liftSpecimen2Timeout:
                if (pathTimer.getElapsedTime() > 250 && arm.isStopped()) {
                    setPathState(observationRetreat2);
                }
                break;
            case observationRetreat2:
                follower.holdPoint(
                        new EasyPoint(poses.Observations.Retreats.Blue),
                        poses.Observations.Retreats.Blue.getHeading());
                setPathState(observationRetreat2Timeout);
                break;
            case observationRetreat2Timeout:
                if (pathTimer.getElapsedTime() > 500 && !follower.isBusy()) {
                    setPathState(toChambers3);
                }
                break;
            case toChambers3:
                follower.followPath(getPath(toChambers3));
                setPathState(chambers3RaiseArm);
                break;
            case chambers3RaiseArm:
                arm.setRotation(PrimaryArm.rotationStates.UP, 4, true);
                grabber.setGrabberState(Grabber.grabberStates.OUT);
                setPathState(chambers3Heading);
                break;
            case chambers3Heading:
                if (follower.getCurrentTValue() > 0.3) {
                    ((EasySafePath) getPath(toChambers3)).setHeading(HeadingTypes.LINEAR, poses.Observations.Grabs.Blue, poses.Chambers.Blue);
                    setPathState(holdChambers3);
                }
                break;
            case holdChambers3:
                if (!follower.isBusy() && arm.isStopped()) {
                    follower.holdPoint(
                            new EasyPoint(poses.Chambers.Blue,
                                    new Offsets().addY(vars.Chassis.FRONT_LENGTH).addY(vars.Mechanisms.Grabber.AtChambers.OUT).remX(vars.sample).remY(1.5)),
                            poses.Chambers.Blue.getHeading()
                    );
                    setPathState(chambers3Timeout);
                }
                break;
            case chambers3Timeout:
                if (pathTimer.getElapsedTime() > 250) {
                    setPathState(chambers3LowerArm);
                }
                break;

            case chambers3LowerArm:
                grabber.setGrabberState(Grabber.grabberStates.HOOK);
                if (pathTimer.getElapsedTime() > 500 && grabber.isSettled()) {
                    arm.setRotation(PrimaryArm.rotationStates.DOWN, 1.25, false);
                    setPathState(chambers3LowerArmTimeout);
                }
                break;
            case chambers3LowerArmTimeout:
                if (pathTimer.getElapsedTime() > 250 && arm.isStopped()) {
                    setPathState(chambers3ReleaseAndBack);
                }
                break;
            case chambers3ReleaseAndBack:
                grabber.release();
                grabber.setGrabberState(Grabber.grabberStates.OUT);
                follower.holdPoint(
                        new EasyPoint(poses.Chambers.Retreats.Blue,
                                new Offsets().addY(vars.Chassis.FRONT_LENGTH).addY(vars.Mechanisms.Grabber.AtChambers.OUT)),
                        poses.Chambers.Retreats.Blue.getHeading()
                );
                setPathState(chambers3ReleaseAndBackTimeout);
                break;
            case chambers3ReleaseAndBackTimeout:
                if (pathTimer.getElapsedTime() > 500) {
                    requestOpModeStop();
//                    setPathState();
                }
                break;
        }
    }

    public void setPathState(Blue_1_2_PathStates state) {
        pathState = state;
        pathTimer.resetTimer();
        autonomousPathUpdate();
    }
}
