package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue;

import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.BluePathStates.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.BotPose.Pinpoint;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.BotPose.PoseHelper;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.BotPose.Vision;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Mechanisms.Grabber.Grabber;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Mechanisms.PrimaryArm.PrimaryArm;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Pathing.Utils.EasyPoint;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Pathing.Utils.O;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Pathing.Utils.Offsets;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Pathing.Utils.Paths.EasySafePath;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Pathing.Utils.Paths.HeadingTypes;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Vars.FieldPoses;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Robots.CompBot.CompBot;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Robots.CompBot.CompBotVars;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.pedroPathing.util.Timer;

import java.util.HashMap;
import java.util.Map;

@Autonomous(name = "A - Blue Auto", group = "Auto")
public class Blue extends OpMode {
    private final CompBot Bot = new CompBot();
    private final CompBotVars vars = new CompBotVars();

    private final Vision vision = new Vision();
    private final Pinpoint pinpoint = new Pinpoint();
    private final PoseHelper pose = new PoseHelper();

    private final FieldPoses poses = new FieldPoses();

    Grabber grabber = new Grabber();

    PrimaryArm arm = new PrimaryArm();

    private Pose startPose;

    private Follower follower;

    private Timer pathTimer;

    private final Map<BluePathStates, Path> paths = new HashMap<>();
    private BluePathStates pathState;

    private final Map<BluePathStates, Double> specimenOffsets = new HashMap<>();

    double pushSampleOffset = 2;

    @Override
    public void init() {
        Bot.initRobot(hardwareMap);

        pinpoint.setOp(this);
        pinpoint.initPinpoint(hardwareMap);

        vision.setOp(this);
        vision.initVision(hardwareMap, pinpoint);

        pose.setOp(this);
        pose.setDevices(vision, pinpoint);

        pathTimer = new Timer();

        vision.start();

        pose.updateLLUsage(false);

        Pose2D currentPose = pose.getSmartPose(PoseHelper.Alliances.BLUE);
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

        arm.initPrimaryArm(hardwareMap, Bot.LinearOp);
        grabber.initGrabber(hardwareMap);

        grabber.close();
        grabber.headStraight();
        grabber.setGrabberState(Grabber.grabberStates.TUCK);
        arm.setRetract();
    }

    public void start() {
        grabber.close();
        grabber.headStraight();
        grabber.setGrabberState(Grabber.grabberStates.TUCK);
        arm.setRetract();
        buildPaths();
        setPathState(toChambers1);
    }

    public void loop() {
        pose.updatePose();
        follower.update();
        grabber.tiltStateCheck();
        arm.rotationChecker();
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

    private Path getPath(BluePathStates state) {
        return O.req(paths.get(state));
    }

    public void buildPaths() {
        paths.put(toChambers1,
                new EasySafePath(startPose, poses.Chambers.Blue,
                        new Offsets().addY(vars.Chassis.FRONT_LENGTH).addY(vars.Mechanisms.Grabber.AtChambers.OUT).addX(vars.sample))
                        .setHeading(HeadingTypes.CONSTANT, startPose));

        paths.put(toSample1,
                new EasySafePath(getPath(toChambers1).getLastControlPoint(), poses.SampleLines.Audience.Blue.B1.Slip, poses.SampleLines.Audience.Blue.B1.Post, poses.SampleLines.Audience.Blue.B1.Sample,
                        new Offsets().remY(vars.Chassis.FRONT_LENGTH))
                        .setHeading(HeadingTypes.LINEAR, getPath(toChambers1), poses.SampleLines.pushApproachAngle, .35));

        paths.put(toObservation1,
                new EasySafePath(getPath(toSample1).getLastControlPoint(), poses.SampleLines.Audience.Blue.B1.Pre, poses.Observations.Grabs.Blue,
                        new Offsets().remY(vars.Mechanisms.Grabber.AtObservation.OUT).addY(pushSampleOffset).remX(pushSampleOffset))
                        .setHeading(HeadingTypes.CONSTANT, poses.Observations.Grabs.Blue));

        paths.put(toChambers2,
                new EasySafePath(getPath(toObservation1).getLastControlPoint(), poses.Observations.Retreats.Blue, poses.Chambers.Blue,
                        new Offsets().addY(vars.Chassis.FRONT_LENGTH).addY(vars.Mechanisms.Grabber.AtChambers.OUT))
                        .setHeading(HeadingTypes.LINEAR, getPath(toObservation1), poses.Chambers.Blue));

        paths.put(toSample2,
                new EasySafePath(getPath(toChambers2).getLastControlPoint(), poses.SampleLines.Audience.Blue.B2.Slip, poses.SampleLines.Audience.Blue.B2.Post, poses.SampleLines.Audience.Blue.B2.Sample,
                        new Offsets().remY(vars.Chassis.FRONT_LENGTH))
                        .setHeading(HeadingTypes.LINEAR, getPath(toChambers2), poses.SampleLines.pushApproachAngle, .35));

        paths.put(toObservation2,
                new EasySafePath(getPath(toSample2).getLastControlPoint(), poses.SampleLines.Audience.Blue.B2.Pre, poses.Observations.Approaches.Blue)
                        .setHeading(HeadingTypes.CONSTANT, poses.Observations.Approaches.Blue));


        paths.put(toChambers3,
                new EasySafePath(getPath(toObservation2).getLastControlPoint(), poses.Observations.Retreats.Blue, poses.Chambers.Blue,
                        new Offsets().addY(vars.Chassis.FRONT_LENGTH).addY(vars.Mechanisms.Grabber.AtChambers.OUT).remX(vars.sample))
                        .setHeading(HeadingTypes.LINEAR, getPath(toObservation2), poses.Chambers.Blue));
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case toChambers1:
                follower.followPath(getPath(toChambers1));
                setPathState(chambers1Heading);
                break;
            case chambers1Heading:
                if (follower.getCurrentTValue() > 0.1) {
                    ((EasySafePath) getPath(toChambers1)).setHeading(HeadingTypes.LINEAR, startPose, poses.Chambers.Blue);
                    setPathState(chambers1RaiseArm);
                }
                break;
            case chambers1RaiseArm:
                arm.setRotation(PrimaryArm.rotationStates.UP, 6, true);
                grabber.setGrabberState(Grabber.grabberStates.OUT);
                setPathState(holdChambers1);
                break;
            case holdChambers1:
                if (!follower.isBusy() && arm.isStopped()) {
                    follower.holdPoint(
                            new EasyPoint(poses.Chambers.Blue,
                                    new Offsets().addY(vars.Chassis.FRONT_LENGTH).addY(vars.Mechanisms.Grabber.AtChambers.OUT).addX(vars.sample)),
                            poses.Chambers.Blue.getHeading()
                    );
                    setPathState(chambers1Timeout);
                }
                break;
            case chambers1Timeout:
                if (pathTimer.getElapsedTime() > 500) {
                    setPathState(chambers1LowerArm);
                }
                break;

            case chambers1LowerArm:
                grabber.setGrabberState(Grabber.grabberStates.HOOK);
                if (pathTimer.getElapsedTime() > 500) {
                    arm.setRotation(PrimaryArm.rotationStates.DOWN, 1, false);
                    setPathState(chambers1LowerArmTimeout);
                }
                break;
            case chambers1LowerArmTimeout:
                if (pathTimer.getElapsedTime() > 500 && arm.isStopped()) {
                    setPathState(chambers1ReleaseAndBack);
                }
                break;
            case chambers1ReleaseAndBack:
                grabber.open();
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
                            new EasyPoint(poses.SampleLines.Audience.Blue.B1.Sample),
                            poses.SampleLines.pushApproachAngle
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

                    setPathState(observationRetreat1);
                }
                break;
            case observationRetreat1:
                follower.holdPoint(
                        new EasyPoint(poses.Observations.Retreats.Blue),
                        poses.Observations.Retreats.Blue.getHeading());

                pose.updateLLUsage(true);
                vision.setPipeline(2);

                setPathState(observationRetreat1Timeout);
                break;
            case observationRetreat1Timeout:
                if (pathTimer.getElapsedTime() > 500 & !follower.isBusy()) {
                    setPathState(alignObservation1);
                }
                break;
            case alignObservation1:
                vision.getResult();

                double[] offsets = vision.getOffsets();
                double currentXOffset = offsets[0];
                double targetXOffset = -20.5;
                double majorCorrectionFactor = 0.25;
                double minorCorrectionFactor = 0.6;
                double majorMinorBreakpoint = 5;

                if (Math.abs(currentXOffset - targetXOffset) < .4) {
                    Pose originalPose = poses.Observations.Approaches.Blue;
                    Pose currentPose = follower.getPose();
                    double xDiff = originalPose.getX() - currentPose.getX();
                    specimenOffsets.put(grabSpecimen1, xDiff);
                    pose.updateLLUsage(false);
                    setPathState(grabberOut1);
                } else {
                    double error = targetXOffset - currentXOffset;

                    double adjustment;

                    if (Math.abs(currentXOffset-targetXOffset) > majorMinorBreakpoint) {
                        adjustment = error * majorCorrectionFactor;
                    } else {
                        adjustment = error * minorCorrectionFactor;
                    }

                    Pose updatedPose = follower.getPose();
                    updatedPose.setX(updatedPose.getX() - adjustment);
                    follower.holdPoint(new EasyPoint(updatedPose), poses.Observations.Approaches.Blue.getHeading());

                    setPathState(alignObservation1Timeout);
                }
                break;
            case alignObservation1Timeout:
                if (pathTimer.getElapsedTime() > 500) {
                    setPathState(alignObservation1);
                }
                break;
            case grabberOut1:
                grabber.setGrabberState(Grabber.grabberStates.OUT);
                if (grabber.isSettled()) {
                    setPathState(grabberOut1Timeout);
                }
                break;
            case grabberOut1Timeout:
                if (pathTimer.getElapsedTime() > 750 && grabber.isSettled()) {
                    setPathState(approachGrabSpecimen1);
                }
                break;
            case approachGrabSpecimen1:
                double a = specimenOffsets.get(grabSpecimen1);
                follower.holdPoint(
                        new EasyPoint(poses.Observations.Grabs.Blue,
                                new Offsets().remX(a).remY(vars.Mechanisms.Grabber.AtObservation.OUT)),
                        poses.Observations.Grabs.Blue.getHeading());
                setPathState(approachGrabSpecimen1Timeout);
                break;
            case approachGrabSpecimen1Timeout:
                if (pathTimer.getElapsedTime() > 750 && !follower.isBusy()) setPathState(grabSpecimen1);
                break;
            case grabSpecimen1:
                grabber.close();
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
                    setPathState(observation1Retreat);
                }
                break;
            case observation1Retreat:
                follower.holdPoint(
                        new EasyPoint(poses.Observations.Retreats.Blue),
                        poses.Observations.Grabs.Blue.getHeading());
                setPathState(observation1RetreatTimeout);
                break;
            case observation1RetreatTimeout:
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
                setPathState(holdChambers2);
                break;
            case holdChambers2:
                if (!follower.isBusy() && arm.isStopped()) {
                    follower.holdPoint(
                            new EasyPoint(poses.Chambers.Blue,
                                    new Offsets().addY(vars.Chassis.FRONT_LENGTH).addY(vars.Mechanisms.Grabber.AtChambers.OUT)),
                            poses.Chambers.Blue.getHeading()
                    );
                    setPathState(chambers2Timeout);
                }
                break;
            case chambers2Timeout:
                if (pathTimer.getElapsedTime() > 500) {
                    setPathState(chambers2LowerArm);
                }
                break;

            case chambers2LowerArm:
                grabber.setGrabberState(Grabber.grabberStates.HOOK);
                if (pathTimer.getElapsedTime() > 500) {
                    arm.setRotation(PrimaryArm.rotationStates.DOWN, 1, false);
                    setPathState(chambers2LowerArmTimeout);
                }
                break;
            case chambers2LowerArmTimeout:
                if (pathTimer.getElapsedTime() > 500 && arm.isStopped()) {
                    setPathState(chambers2ReleaseAndBack);
                }
                break;
            case chambers2ReleaseAndBack:
                grabber.open();
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
                    follower.holdPoint(
                            new EasyPoint(poses.SampleLines.Audience.Blue.B2.Sample),
                            poses.SampleLines.pushApproachAngle
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
                            new EasyPoint(poses.Observations.Approaches.Blue),
                            poses.Observations.Approaches.Blue.getHeading()
                    );

                    pose.updateLLUsage(true);
                    vision.setPipeline(2);

                    setPathState(observation2Timeout);
                }
                break;
            case observation2Timeout:
                if (pathTimer.getElapsedTime() > 750 && !follower.isBusy()) {
                    setPathState(alignObservation2);
                }
                break;
            case alignObservation2:
                vision.getResult();

                double[] offsets2 = vision.getOffsets();
                double currentXOffset2 = offsets2[0];
                double targetXOffset2 = -20.5;
                double majorCorrectionFactor2 = 0.25;
                double minorCorrectionFactor2 = 0.6;
                double majorMinorBreakpoint2 = 5;

                if (Math.abs(currentXOffset2 - targetXOffset2) < .2) {
                    Pose originalPose2 = poses.Observations.Approaches.Blue;
                    Pose currentPose2 = follower.getPose();
                    double xDiff2 = originalPose2.getX() - currentPose2.getX();
                    specimenOffsets.put(grabSpecimen2, xDiff2);
                    pose.updateLLUsage(false);
                    setPathState(approachPushSample2);
                } else {
                    double error2 = targetXOffset2 - currentXOffset2;

                    double adjustment2;

                    if (Math.abs(currentXOffset2-targetXOffset2) > majorMinorBreakpoint2) {
                        adjustment2 = error2 * majorCorrectionFactor2;
                    } else {
                        adjustment2 = error2 * minorCorrectionFactor2;
                    }

                    Pose updatedPose2 = follower.getPose();
                    updatedPose2.setX(updatedPose2.getX() - adjustment2);
                    follower.holdPoint(new EasyPoint(updatedPose2), poses.Observations.Approaches.Blue.getHeading());

                    setPathState(alignObservation2Timeout);
                }
                break;
            case alignObservation2Timeout:
                if (pathTimer.getElapsedTime() > 500) {
                    setPathState(alignObservation2);
                }
                break;
            case approachPushSample2:
                grabber.setGrabberState(Grabber.grabberStates.DOWN);
                double a3 = specimenOffsets.get(grabSpecimen2);
                follower.holdPoint(
                        new EasyPoint(poses.Observations.Grabs.Blue,
                                new Offsets().remX(a3).remY(vars.Mechanisms.Grabber.AtObservation.OUT).addY(pushSampleOffset).remX(pushSampleOffset)),
                        poses.Observations.Grabs.Blue.getHeading());
                setPathState(approachPushSample2Timeout);
                break;
            case approachPushSample2Timeout:
                if (pathTimer.getElapsedTime() > 500 && !follower.isBusy()) {
                    setPathState(retreatPushSample2);
                }
                break;
            case retreatPushSample2:
                grabber.setGrabberState(Grabber.grabberStates.DOWN);
                double a4 = specimenOffsets.get(grabSpecimen2);
                follower.holdPoint(
                        new EasyPoint(poses.Observations.Grabs.Blue,
                                new Offsets().remX(a4).remY(vars.Mechanisms.Grabber.AtObservation.OUT).remY(pushSampleOffset)),
                        poses.Observations.Grabs.Blue.getHeading());
                setPathState(retreatPushSample2Timeout);
                break;
            case retreatPushSample2Timeout:
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
                if (pathTimer.getElapsedTime() > 750 && grabber.isSettled()) {
                    setPathState(approachGrabSpecimen2);
                }
                break;
            case approachGrabSpecimen2:
                double a5 = specimenOffsets.get(grabSpecimen2);
                follower.holdPoint(
                        new EasyPoint(poses.Observations.Grabs.Blue,
                                new Offsets().remX(a5).remY(vars.Mechanisms.Grabber.AtObservation.OUT)),
                        poses.Observations.Grabs.Blue.getHeading());
                setPathState(approachGrabSpecimen2Timeout);
                break;
            case approachGrabSpecimen2Timeout:
                if (pathTimer.getElapsedTime() > 750 && !follower.isBusy()) setPathState(grabSpecimen2);
                break;
            case grabSpecimen2:
                grabber.close();
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
                    setPathState(observation2Retreat);
                }
                break;
            case observation2Retreat:
                follower.holdPoint(
                        new EasyPoint(poses.Observations.Retreats.Blue),
                        poses.Observations.Grabs.Blue.getHeading());
                setPathState(observation2RetreatTimeout);
                break;
            case observation2RetreatTimeout:
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
                setPathState(holdChambers3);
                break;
            case holdChambers3:
                if (!follower.isBusy() && arm.isStopped()) {
                    follower.holdPoint(
                            new EasyPoint(poses.Chambers.Blue,
                                    new Offsets().addY(vars.Chassis.FRONT_LENGTH).addY(vars.Mechanisms.Grabber.AtChambers.OUT)),
                            poses.Chambers.Blue.getHeading()
                    );
                    setPathState(chambers3Timeout);
                }
                break;
            case chambers3Timeout:
                if (pathTimer.getElapsedTime() > 500) {
                    setPathState(chambers3LowerArm);
                }
                break;

            case chambers3LowerArm:
                grabber.setGrabberState(Grabber.grabberStates.HOOK);
                if (pathTimer.getElapsedTime() > 500) {
                    arm.setRotation(PrimaryArm.rotationStates.DOWN, 1, false);
                    setPathState(chambers3LowerArmTimeout);
                }
                break;
            case chambers3LowerArmTimeout:
                if (pathTimer.getElapsedTime() > 500 && arm.isStopped()) {
                    setPathState(chambers3ReleaseAndBack);
                }
                break;
            case chambers3ReleaseAndBack:
                grabber.open();
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
//                    setPathState();
                }
                break;
        }
    }

    public void setPathState(BluePathStates state) {
        pathState = state;
        pathTimer.resetTimer();
        autonomousPathUpdate();
    }
}
