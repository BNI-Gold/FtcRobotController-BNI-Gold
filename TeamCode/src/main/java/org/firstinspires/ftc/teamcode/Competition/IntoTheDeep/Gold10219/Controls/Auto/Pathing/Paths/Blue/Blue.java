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

    @Override
    public void init() {
        Bot.initRobot(hardwareMap);

        arm.initPrimaryArm(hardwareMap, Bot.LinearOp);
        grabber.initGrabber(hardwareMap);

        pinpoint.setOp(this);
        pinpoint.initPinpoint(hardwareMap);

        vision.setOp(this);
        vision.initVision(hardwareMap, pinpoint);

        pose.setOp(this);
        pose.setDevices(vision, pinpoint);

        pathTimer = new Timer();

        vision.start();

        pose.updateLLUsage(false);

        follower = new Follower(hardwareMap);
    }

    public void start() {
        grabber.close();
        grabber.headStraight();
        grabber.setGrabberState(Grabber.grabberStates.TUCK);
        arm.setRetract();
        setPathState(raiseArm);
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

    private Path getPath(BluePathStates state) {
        return O.req(paths.get(state));
    }

    public void buildPaths() {
        paths.put(toChambers1,
                new EasySafePath(startPose, poses.Chambers.Blue,
                        new Offsets().addY(vars.Chassis.FRONT_LENGTH).addY(vars.Mechanisms.Grabber.GRABBER_HOOK_POSITION))
                        .setHeading(HeadingTypes.CONSTANT, startPose));

        paths.put(toSample1,
                new EasySafePath(getPath(toChambers1).getLastControlPoint(), poses.SampleLines.Audience.Blue.B1.Slip, poses.SampleLines.Audience.Blue.B1.Post, poses.SampleLines.Audience.Blue.B1.Sample,
                        new Offsets().remY(vars.Chassis.FRONT_LENGTH))
                        .setHeading(HeadingTypes.LINEAR, getPath(toChambers1), poses.SampleLines.pushApproachAngle, .35));

        paths.put(toRecal1,
                new EasySafePath(getPath(toSample1).getLastControlPoint(), poses.SampleLines.Audience.Blue.B1.Pre, poses.Recalibration.Single.A11)
                        .setHeading(HeadingTypes.CONSTANT, poses.Recalibration.Single.A11));

        paths.put(toObservation1,
                new EasySafePath(getPath(toRecal1).getLastControlPoint(), poses.Observations.Approaches.Blue)
                        .setHeading(HeadingTypes.CONSTANT, poses.Observations.Approaches.Blue));

        paths.put(toSample2,
                new EasySafePath(getPath(toObservation1).getLastControlPoint(), poses.Observations.Retreats.Blue, poses.SampleLines.Audience.Blue.B2.Pre, poses.SampleLines.Audience.Blue.B2.Post, poses.SampleLines.Audience.Blue.B2.Sample,
                        new Offsets().remY(vars.Chassis.FRONT_LENGTH))
                        .setHeading(HeadingTypes.CONSTANT, poses.SampleLines.pushApproachAngle));

        paths.put(toObservation2,
                new EasySafePath(getPath(toSample2).getLastControlPoint(), poses.Observations.Blue)
                        .setHeading(HeadingTypes.CONSTANT, poses.Observations.Blue));

        paths.put(toChambers2,
                new EasySafePath(getPath(toObservation2).getLastControlPoint(), poses.Chambers.Blue,
                        new Offsets().addY(vars.Chassis.FRONT_LENGTH).addY(vars.Mechanisms.Grabber.GRABBER_HOOK_POSITION))
                        .setHeading(HeadingTypes.LINEAR, getPath(toObservation2), poses.Chambers.Blue));

        paths.put(toObservation3,
                new EasySafePath(getPath(toChambers2).getLastControlPoint(), poses.Observations.Blue,
                        new Offsets().remY(vars.Chassis.FRONT_LENGTH).remY(vars.Mechanisms.Grabber.GRABBER_EXTENDED_POSITION))
                        .setHeading(HeadingTypes.LINEAR, getPath(toChambers2), poses.Observations.Blue));

        paths.put(toChambers3,
                new EasySafePath(getPath(toObservation3).getLastControlPoint(), poses.Chambers.Midpoints.Blue, poses.Chambers.Blue,
                        new Offsets().addY(vars.Chassis.FRONT_LENGTH).addY(vars.Mechanisms.Grabber.GRABBER_HOOK_POSITION))
                        .setHeading(HeadingTypes.LINEAR, getPath(toObservation3), poses.Chambers.Blue));

        paths.put(toObservation4,
                new EasySafePath(getPath(toChambers3).getLastControlPoint(), poses.Observations.Blue,
                        new Offsets().remY(vars.Chassis.FRONT_LENGTH).remY(vars.Mechanisms.Grabber.GRABBER_EXTENDED_POSITION))
                        .setHeading(HeadingTypes.LINEAR, getPath(toChambers3), poses.Observations.Blue));

        paths.put(toChambers4,
                new EasySafePath(getPath(toObservation4).getLastControlPoint(), poses.Chambers.Midpoints.Blue, poses.Chambers.Blue,
                        new Offsets().addY(vars.Chassis.FRONT_LENGTH).addY(vars.Mechanisms.Grabber.GRABBER_HOOK_POSITION))
                        .setHeading(HeadingTypes.LINEAR, getPath(toObservation4), poses.Chambers.Blue));
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case raiseArm:
                arm.up(2, true);
                setPathState(raiseArmTimeout);
                break;
            case raiseArmTimeout:
                if (pathTimer.getElapsedTime() > 500) {
                    setPathState(getPosition);
                }
                break;
            case getPosition: {
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

                follower.setStartingPose(startPose);
                setPathState(getGetPositionTimeout);
                break;
            }
            case getGetPositionTimeout:
                if (pathTimer.getElapsedTime() > 500) {
                    setPathState(buildPaths);
                }
                break;
            case buildPaths:
                buildPaths();
                setPathState(buildPathsTimeout);
                break;
            case buildPathsTimeout:
                if (pathTimer.getElapsedTime() > 500) {
                    setPathState(toChambers1);
                }
                break;
            case toChambers1:
                follower.followPath(getPath(toChambers1));
                Path p = getPath(toChambers1);
                Pose po = follower.getPose();
                telemetry.addData("PFX: ", p.getFirstControlPoint().getX());
                telemetry.addData("PFY: ", p.getFirstControlPoint().getY());
                telemetry.addData("PLX: ", p.getLastControlPoint().getX());
                telemetry.addData("PLY: ", p.getLastControlPoint().getY());
                telemetry.addLine();
                telemetry.addData("Pose X: ", po.getX());
                telemetry.addData("Pose Y: ", po.getY());
                setPathState(chambers1Heading);
                break;
            case chambers1Heading:
                if (follower.getCurrentTValue() > 0.1) {
                    ((EasySafePath) getPath(toChambers1)).setHeading(HeadingTypes.LINEAR, startPose, poses.Chambers.Blue);
                    setPathState(holdChambers1);
                }
                break;
            case holdChambers1:
                if (!follower.isBusy()) {
                    follower.holdPoint(
                            new EasyPoint(poses.Chambers.Blue,
                                    new Offsets().addY(vars.Chassis.FRONT_LENGTH).addY(vars.Mechanisms.Grabber.GRABBER_HOOK_POSITION)),
                            poses.Chambers.Blue.getHeading()
                    );
                    setPathState(chambers1Timeout);
                }
                break;
            case chambers1Timeout:
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
                    follower.holdPoint(
                            new EasyPoint(poses.SampleLines.Audience.Blue.B1.Sample),
                            poses.SampleLines.pushApproachAngle
                    );
                    setPathState(toRecal1);
                }
                break;
            case toRecal1:
                follower.followPath(getPath(toRecal1));
                setPathState(holdRecal1);
                break;
            case holdRecal1:
                if (!follower.isBusy()) {
                    follower.holdPoint(
                            new EasyPoint(poses.Recalibration.Single.A11),
                            poses.Recalibration.Single.A11.getHeading()
                    );
                    setPathState(recal1Timeout);
                }
                break;
            case recal1Timeout:
                setPathState(toObservation1);
                break;
            case toObservation1:
                follower.followPath(getPath(toObservation1));
                setPathState(holdObservation1);
                break;
            case holdObservation1:
                if (!follower.isBusy()) {
                    follower.holdPoint(
                            new EasyPoint(poses.Observations.Approaches.Blue),
                            poses.Observations.Approaches.Blue.getHeading()
                    );

                    pose.updateLLUsage(true);
                    vision.setPipeline(2);

                    setPathState(observation1Timeout);
                }
                break;
            case observation1Timeout:
                if (pathTimer.getElapsedTime() > 500) {
                    setPathState(alignObservation1);
                }
                break;
            case alignObservation1: {
                vision.getResult();

                double[] offsets = vision.getOffsets();
                double currentXOffset = offsets[0];
                double targetXOffset = 2.15;
                double majorCorrectionFactor = 0.25;
                double minorCorrectionFactor = 0.6;
                double majorMinorBreakpoint = 5;

                if (Math.abs(currentXOffset - targetXOffset) < .4) {
                    Pose originalPose = poses.Observations.Approaches.Blue;
                    Pose currentPose = follower.getPose();
                    double xDiff = originalPose.getX() - currentPose.getX();
                    specimenOffsets.put(grabSpecimen1, xDiff);
                    pose.updateLLUsage(false);
                    setPathState(approachGrabSpecimen1);
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
            }
            case alignObservation1Timeout:
                if (pathTimer.getElapsedTime() > 500) {
                    setPathState(alignObservation1);
                }
                break;
            case approachGrabSpecimen1:
                double a = specimenOffsets.get(grabSpecimen1);
                Pose updatedPose = poses.Observations.Grabs.Blue;
                updatedPose.setX(poses.Observations.Grabs.Blue.getX() - a);
                follower.holdPoint(new EasyPoint(updatedPose), poses.Observations.Grabs.Blue.getHeading());
                setPathState(approachGrabSpecimen1Timeout);
                break;
            case approachGrabSpecimen1Timeout:
                if (pathTimer.getElapsedTime() > 1000) setPathState(grabSpecimen1);
                break;
            case grabSpecimen1:
//                setPathState(grabSpecimen1Timeout);
                break;
            case grabSpecimen1Timeout:
                setPathState(toSample2);
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
                    l = follower.getPose();
                    setPathState(toObservation2);
                }
                break;
            case toObservation2:
                follower.followPath(getPath(toObservation2));
                setPathState(holdObservation2);
                break;
            case holdObservation2:
                if (!follower.isBusy()) {
                    follower.holdPoint(
                            new EasyPoint(poses.Observations.Blue),
                            poses.Observations.Blue.getHeading()
                    );
                    setPathState(observation2Timeout);
                }
                break;
            case observation2Timeout:
                if (pathTimer.getElapsedTime() > 500) {
                    setPathState(toChambers2);
                }
                break;
            case toChambers2:
                follower.followPath(getPath(toChambers2));
                setPathState(holdChambers2);
                break;
            case holdChambers2:
                if (!follower.isBusy()) {
                    follower.holdPoint(
                            new EasyPoint(poses.Chambers.Blue,
                                    new Offsets().addY(vars.Chassis.FRONT_LENGTH).addY(vars.Mechanisms.Grabber.GRABBER_HOOK_POSITION)),
                            poses.Chambers.Blue.getHeading()
                    );
                    setPathState(chambers2Timeout);
                }
                break;
            case chambers2Timeout:
                if (pathTimer.getElapsedTime() > 500) {
                    setPathState(toObservation3);
                }
                break;
            case toObservation3:
                follower.followPath(getPath(toObservation3));
                setPathState(holdObservation3);
                break;
            case holdObservation3:
                if (!follower.isBusy()) {
                    follower.holdPoint(
                            new EasyPoint(poses.Observations.Blue),
                            poses.Observations.Blue.getHeading()
                    );
                    setPathState(observation3Timeout);
                }
                break;
            case observation3Timeout:
                if (pathTimer.getElapsedTime() > 500) {
                    setPathState(toChambers3);
                }
                break;
            case toChambers3:
                follower.followPath(getPath(toChambers3));
                setPathState(holdChambers3);
                break;
            case holdChambers3:
                if (!follower.isBusy()) {
                    follower.holdPoint(
                            new EasyPoint(poses.Chambers.Blue,
                                    new Offsets().addY(vars.Chassis.FRONT_LENGTH).addY(vars.Mechanisms.Grabber.GRABBER_HOOK_POSITION)),
                            poses.Chambers.Blue.getHeading()
                    );
                    setPathState(chambers3Timeout);
                }
                break;
            case chambers3Timeout:
                if (pathTimer.getElapsedTime() > 500) {
                    setPathState(toObservation4);
                }
                break;
            case toObservation4:
                follower.followPath(getPath(toObservation4));
                setPathState(holdObservation4);
                break;
            case holdObservation4:
                if (!follower.isBusy()) {
                    follower.holdPoint(
                            new EasyPoint(poses.Observations.Blue),
                            poses.Observations.Blue.getHeading()
                    );
                    setPathState(observation4Timeout);
                }
                break;
            case observation4Timeout:
                if (pathTimer.getElapsedTime() > 500) {
                    setPathState(toChambers4);
                }
                break;
            case toChambers4:
                follower.followPath(getPath(toChambers4));
                setPathState(holdChambers4);
                break;
            case holdChambers4:
                if (!follower.isBusy()) {
                    follower.holdPoint(
                            new EasyPoint(poses.Chambers.Blue,
                                    new Offsets().addY(vars.Chassis.FRONT_LENGTH).addY(vars.Mechanisms.Grabber.GRABBER_HOOK_POSITION)),
                            poses.Chambers.Blue.getHeading()
                    );
                    setPathState(chambers4Timeout);
                }
                break;
            case chambers4Timeout:
                if (pathTimer.getElapsedTime() > 500) {
//                    setPathState(toObservation4);
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
