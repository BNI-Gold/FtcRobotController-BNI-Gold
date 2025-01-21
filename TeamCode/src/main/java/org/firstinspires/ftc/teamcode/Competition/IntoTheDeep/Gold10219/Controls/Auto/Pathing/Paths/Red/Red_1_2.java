package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red;

import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.approachGrabSpecimen1;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.approachGrabSpecimen1Timeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.approachGrabSpecimen2;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.approachGrabSpecimen2Timeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.chambers1Heading;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.chambers1LowerArm;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.chambers1LowerArmTimeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.chambers1RaiseArm;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.chambers1ReleaseAndBack;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.chambers1ReleaseAndBackTimeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.chambers1Timeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.chambers2Heading;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.chambers2LowerArm;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.chambers2LowerArmTimeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.chambers2RaiseArm;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.chambers2ReleaseAndBack;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.chambers2ReleaseAndBackTimeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.chambers2Timeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.chambers3Heading;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.chambers3LowerArm;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.chambers3LowerArmTimeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.chambers3RaiseArm;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.chambers3ReleaseAndBack;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.chambers3ReleaseAndBackTimeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.chambers3Timeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.grabSpecimen1;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.grabSpecimen1Timeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.grabSpecimen2;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.grabSpecimen2Timeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.grabberOut1;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.grabberOut1Timeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.grabberOut2;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.grabberOut2Timeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.holdChambers1;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.holdChambers2;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.holdChambers3;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.holdObservation1;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.holdObservation1Timeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.holdObservation2;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.holdObservation2Timeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.holdSample1;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.holdSample2;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.liftSpecimen1;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.liftSpecimen1Timeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.liftSpecimen2;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.liftSpecimen2Timeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.observationRetreat1;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.observationRetreat1Timeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.observationRetreat2;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.observationRetreat2Timeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.observationRetreatFromPush1;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.observationRetreatFromPush1Timeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.observationRetreatFromPush2;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.observationRetreatFromPush2Timeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.toChambers1;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.toChambers2;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.toChambers3;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.toObservation1;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.toObservation2;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.toSample1;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Red_1_2_PathStates.toSample2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Path;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Vars.FieldPoses;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Mechanisms.Grabber.Grabber;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Mechanisms.Outgrabber.Outgrabber;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Mechanisms.PrimaryArm.PrimaryArm;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Mechanisms.SecondaryArm.SecondaryArm;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.PedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.PedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Robots.CompBot.CompBot;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Robots.CompBot.CompBotVars;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Utils.EasyPoint;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Utils.O;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Utils.Offsets;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Utils.Paths.EasySafePath;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Utils.Paths.HeadingTypes;

import java.util.HashMap;
import java.util.Map;

@Autonomous(name = "A - Red 1 + 2", group = "Auto - Red")
public class Red_1_2 extends OpMode {
    private final CompBot Bot = new CompBot();
    private final CompBotVars vars = new CompBotVars();

    private final FieldPoses poses = new FieldPoses();

    Grabber grabber = new Grabber();

    PrimaryArm primaryArm = new PrimaryArm();

    Outgrabber outgrabber = new Outgrabber();

    SecondaryArm secondaryArm = new SecondaryArm();

    private Pose startPose;

    private Follower follower;

    private Timer pathTimer, overallTimer;

    private final Map<Red_1_2_PathStates, Path> paths = new HashMap<>();
    private Red_1_2_PathStates pathState;
    private Red_1_2_PathStates savedPathState;

    double pushSampleOffset = 2;

    double targetXOffset = -20.5;
    double majorCorrectionFactor = 0.25;
    double minorCorrectionFactor = 0.6;
    double majorMinorBreakpoint = 5;
    double permissibleOffset = .75;

    @Override
    public void init() {
        Bot.initRobot(hardwareMap);

        pathTimer = new Timer();
        overallTimer = new Timer();

        startPose = new Pose(96, 8, Math.toRadians(180));

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Start Pose: ", startPose);
        telemetry.update();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        primaryArm.initPrimaryArm(hardwareMap, Bot.LinearOp);
        grabber.initGrabber(hardwareMap);

        secondaryArm.initSecondaryArm(hardwareMap, Bot.LinearOp);
        outgrabber.initOutgrabber(hardwareMap);

        grabber.grab();
        grabber.headStraight();
        grabber.setGrabberState(Grabber.grabberStates.TUCK);
        primaryArm.setRetract();

        outgrabber.grab();
        outgrabber.headStraight();
        secondaryArm.setRetract();
    }

    public void start() {
        grabber.grab();
        grabber.headStraight();
        grabber.setGrabberState(Grabber.grabberStates.TUCK);
        primaryArm.setRetract();

        outgrabber.grab();
        outgrabber.headStraight();
        secondaryArm.setRetract();

        buildPaths();
        setPathState(toChambers1);
    }

    private double totalTime = 0;

    public void loop() {
        follower.update();
        grabber.tiltStateCheck();
        primaryArm.positionChecker();
        if (totalTime == 0.0) {
            if ((grabber.imu.getSystemStatus() == BNO055IMU.SystemStatus.SYSTEM_ERROR || grabber.imu.getSystemStatus() == BNO055IMU.SystemStatus.IDLE) && pathState != Red_1_2_PathStates.STOP_FOR_IMU_RESET) {
                savedPathState = pathState;               // remember where you were
                setPathState(Red_1_2_PathStates.STOP_FOR_IMU_RESET);
                return;
            }

            autonomousPathUpdate();
            tel();
        } else {
            telemetry.addData("Total Time: ", totalTime);
        }
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

    private Path getPath(Red_1_2_PathStates state) {
        return O.req(paths.get(state));
    }

    public void buildPaths() {
        paths.put(toChambers1, new EasySafePath(startPose, poses.Chambers.Red, new Offsets().remY(vars.Chassis.FRONT_LENGTH).remY(vars.Mechanisms.Grabber.AtChambers.OUT).remX(vars.sample)).setHeading(HeadingTypes.CONSTANT, startPose));

        paths.put(toSample1, new EasySafePath(getPath(toChambers1).getLastControlPoint(), poses.SampleLines.Red.R1.Slip, poses.SampleLines.Red.R1.Post, poses.SampleLines.Red.R1.Sample, new Offsets().addY(vars.Chassis.FRONT_LENGTH)).setHeading(HeadingTypes.LINEAR, getPath(toChambers1), poses.SampleLines.Red.pushApproachAngle, .35));

        paths.put(toObservation1, new EasySafePath(getPath(toSample1).getLastControlPoint(), poses.SampleLines.Red.R1.Pre, poses.Observations.Grabs.Red, new Offsets().addY(vars.Mechanisms.Grabber.AtObservation.OUT).remY(pushSampleOffset).addX(pushSampleOffset)).setHeading(HeadingTypes.CONSTANT, poses.Observations.Grabs.Red));

        paths.put(toChambers2, new EasySafePath(getPath(toObservation1).getLastControlPoint(), poses.Observations.Retreats.Red, poses.Chambers.Red, new Offsets().remY(vars.Chassis.FRONT_LENGTH).remY(vars.Mechanisms.Grabber.AtChambers.OUT)).setHeading(HeadingTypes.CONSTANT, poses.Observations.Grabs.Red));

        paths.put(toSample2, new EasySafePath(getPath(toChambers2).getLastControlPoint(), poses.SampleLines.Red.R2.Slip, poses.SampleLines.Red.R2.Post, poses.SampleLines.Red.R2.Sample, new Offsets().addY(vars.Chassis.FRONT_LENGTH)).setHeading(HeadingTypes.LINEAR, getPath(toChambers2), poses.SampleLines.Red.pushApproachAngle, .35));

        paths.put(toObservation2, new EasySafePath(getPath(toSample2).getLastControlPoint(), poses.SampleLines.Red.R2.Pre, poses.Observations.Grabs.Red, new Offsets().addY(vars.Mechanisms.Grabber.AtObservation.OUT).remY(pushSampleOffset).addX(pushSampleOffset)).setHeading(HeadingTypes.CONSTANT, poses.Observations.Grabs.Red));

        paths.put(toChambers3, new EasySafePath(getPath(toObservation2).getLastControlPoint(), poses.Observations.Retreats.Red, poses.Chambers.Red, new Offsets().remY(vars.Chassis.FRONT_LENGTH).remY(vars.Mechanisms.Grabber.AtChambers.OUT).addX(vars.sample)).setHeading(HeadingTypes.CONSTANT, poses.Observations.Grabs.Red));
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case STOP_FOR_IMU_RESET:
                // Fully stop. If your Follower has a “stop()” or “holdPoint()”:
//                follower.holdPoint(follower.getPose());
                // or Bot.drive(0,0,0), etc.

                // Re-init IMU if you haven’t already, but only if you’re stationary
                if (pathTimer.getElapsedTime() > 300 && !follower.isBusy()) { // small wait
                    grabber.initializeIMU();
                    pathTimer.resetTimer();
                }

                if (grabber.imu.getSystemStatus() == BNO055IMU.SystemStatus.RUNNING_FUSION) {
                    // done, resume
                    setPathState(savedPathState);
                }
                break;
            case toChambers1:
                follower.followPath(getPath(toChambers1));
                grabber.setGrabberState(Grabber.grabberStates.TUCK);
                setPathState(chambers1Heading);
                break;
            case chambers1Heading:
                if (follower.getCurrentTValue() > 0.1) {
                    ((EasySafePath) getPath(toChambers1)).setHeading(HeadingTypes.LINEAR, startPose, poses.Chambers.Red);
                    setPathState(chambers1RaiseArm);
                }
                break;
            case chambers1RaiseArm:
                primaryArm.setPosition(PrimaryArm.positionStates.ALIGN_SPECIMEN, true);
                outgrabber.midPosition();
                grabber.setGrabberState(Grabber.grabberStates.OUT);
                setPathState(holdChambers1);
                break;
            case holdChambers1:
                if (!follower.isBusy() && primaryArm.isStopped() && grabber.isAtTargetAngle()) {
                    follower.holdPoint(new EasyPoint(poses.Chambers.Red, new Offsets().remY(vars.Chassis.FRONT_LENGTH).remY(vars.Mechanisms.Grabber.AtChambers.OUT).remX(vars.sample)), poses.Chambers.Red.getHeading());
                    setPathState(chambers1Timeout);
                }
                break;
            case chambers1Timeout:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 500 && grabber.isAtTargetAngle()) {
                    setPathState(chambers1LowerArm);
                }
                break;

            case chambers1LowerArm:
                grabber.setGrabberState(Grabber.grabberStates.HOOK);
                if (pathTimer.getElapsedTime() > 500 && grabber.isAtTargetAngle()) {
                    primaryArm.setPosition(PrimaryArm.positionStates.HOOK_SPECIMEN, true);
                    setPathState(chambers1LowerArmTimeout);
                }
                break;
            case chambers1LowerArmTimeout:
                if (pathTimer.getElapsedTime() > 250 && primaryArm.isStopped()) {
                    setPathState(chambers1ReleaseAndBack);
                }
                break;
            case chambers1ReleaseAndBack:
                grabber.release();
                grabber.setGrabberState(Grabber.grabberStates.OUT);
                follower.holdPoint(new EasyPoint(poses.Chambers.Retreats.Red, new Offsets().remY(vars.Chassis.FRONT_LENGTH).remY(vars.Mechanisms.Grabber.AtChambers.OUT)), poses.Chambers.Retreats.Red.getHeading());
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
                    follower.holdPoint(new EasyPoint(poses.SampleLines.Red.R1.Sample), poses.SampleLines.Red.pushApproachAngle);
                    setPathState(toObservation1);
                }
                break;
            case toObservation1:
                follower.followPath(getPath(toObservation1));
                setPathState(holdObservation1);
                break;
            case holdObservation1:
                if (!follower.isBusy()) {
                    primaryArm.setPosition(PrimaryArm.positionStates.GRAB_SPECIMEN, true);
                    follower.holdPoint(new EasyPoint(poses.Observations.Grabs.Red, new Offsets().addY(vars.Mechanisms.Grabber.AtObservation.OUT).remY(pushSampleOffset).addX(pushSampleOffset)), poses.Observations.Grabs.Red.getHeading());

                    setPathState(holdObservation1Timeout);
                }
                break;
            case holdObservation1Timeout:
                if (pathTimer.getElapsedTime() > 500 && !follower.isBusy()) {
                    setPathState(observationRetreatFromPush1);
                }
                break;
            case observationRetreatFromPush1:
                follower.holdPoint(new EasyPoint(poses.Observations.Approaches.Red), poses.Observations.Approaches.Red.getHeading());

                setPathState(observationRetreatFromPush1Timeout);
                break;
            case observationRetreatFromPush1Timeout:
                if (pathTimer.getElapsedTime() > 500 & !follower.isBusy()) {
                    setPathState(grabberOut1);
                }
                break;
            case grabberOut1:
                grabber.setGrabberState(Grabber.grabberStates.OUT);
                if (grabber.isAtTargetAngle()) {
                    setPathState(grabberOut1Timeout);
                }
                break;
            case grabberOut1Timeout:
                if (pathTimer.getElapsedTime() > 250 && grabber.isAtTargetAngle()) {
                    setPathState(approachGrabSpecimen1);
                }
                break;
            case approachGrabSpecimen1:
                follower.holdPoint(new EasyPoint(poses.Observations.Grabs.Red, new Offsets().addY(vars.Mechanisms.Grabber.AtObservation.OUT)), poses.Observations.Grabs.Red.getHeading());
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
                primaryArm.setPosition(PrimaryArm.positionStates.RAISE_SPECIMEN, false);
                setPathState(liftSpecimen1Timeout);
                break;
            case liftSpecimen1Timeout:
                if (pathTimer.getElapsedTime() > 250 && primaryArm.isStopped()) {
                    setPathState(observationRetreat1);
                }
                break;
            case observationRetreat1:
                follower.holdPoint(new EasyPoint(poses.Observations.Retreats.Red), poses.Observations.Retreats.Red.getHeading());
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
                primaryArm.setPosition(PrimaryArm.positionStates.ALIGN_SPECIMEN, true);
                grabber.setGrabberState(Grabber.grabberStates.OUT);
                setPathState(chambers2Heading);
                break;
            case chambers2Heading:
                if (follower.getCurrentTValue() > 0.3) {
                    ((EasySafePath) getPath(toChambers2)).setHeading(HeadingTypes.LINEAR, poses.Observations.Grabs.Red, poses.Chambers.Red);
                    setPathState(holdChambers2);
                }
                break;
            case holdChambers2:
                if (!follower.isBusy() && primaryArm.isStopped()) {
                    follower.holdPoint(new EasyPoint(poses.Chambers.Red, new Offsets().remY(vars.Chassis.FRONT_LENGTH).remY(vars.Mechanisms.Grabber.AtChambers.OUT).addY(.75)), poses.Chambers.Red.getHeading());
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
                if (pathTimer.getElapsedTime() > 500 && grabber.isAtTargetAngle()) {
                    primaryArm.setPosition(PrimaryArm.positionStates.HOOK_SPECIMEN, true);
                    setPathState(chambers2LowerArmTimeout);
                }
                break;
            case chambers2LowerArmTimeout:
                if (pathTimer.getElapsedTime() > 200 && primaryArm.isStopped()) {
                    setPathState(chambers2ReleaseAndBack);
                }
                break;
            case chambers2ReleaseAndBack:
                grabber.release();
                grabber.setGrabberState(Grabber.grabberStates.OUT);
                follower.holdPoint(new EasyPoint(poses.Chambers.Retreats.Red, new Offsets().remY(vars.Chassis.FRONT_LENGTH).remY(vars.Mechanisms.Grabber.AtChambers.OUT)), poses.Chambers.Retreats.Red.getHeading());
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
                    follower.holdPoint(new EasyPoint(poses.SampleLines.Red.R2.Sample), poses.SampleLines.Red.pushApproachAngle);
                    setPathState(toObservation2);
                }
                break;
            case toObservation2:
                follower.followPath(getPath(toObservation2));
                setPathState(holdObservation2);
                break;
            case holdObservation2:
                if (!follower.isBusy()) {
                    primaryArm.setPosition(PrimaryArm.positionStates.GRAB_SPECIMEN, true);
                    follower.holdPoint(new EasyPoint(poses.Observations.Grabs.Red, new Offsets().addY(vars.Mechanisms.Grabber.AtObservation.OUT).remY(pushSampleOffset).addX(pushSampleOffset)), poses.Observations.Grabs.Red.getHeading());

                    setPathState(holdObservation2Timeout);
                }
                break;
            case holdObservation2Timeout:
                if (pathTimer.getElapsedTime() > 500 && !follower.isBusy()) {
                    setPathState(observationRetreatFromPush2);
                }
                break;
            case observationRetreatFromPush2:
                follower.holdPoint(new EasyPoint(poses.Observations.Approaches.Red), poses.Observations.Approaches.Red.getHeading());

                setPathState(observationRetreatFromPush2Timeout);
                break;
            case observationRetreatFromPush2Timeout:
                if (pathTimer.getElapsedTime() > 500 & !follower.isBusy()) {
                    setPathState(grabberOut2);
                }
                break;
            case grabberOut2:
                grabber.setGrabberState(Grabber.grabberStates.OUT);
                if (grabber.isAtTargetAngle()) {
                    setPathState(grabberOut2Timeout);
                }
                break;
            case grabberOut2Timeout:
                if (pathTimer.getElapsedTime() > 250 && grabber.isAtTargetAngle()) {
                    setPathState(approachGrabSpecimen2);
                }
                break;
            case approachGrabSpecimen2:
                follower.holdPoint(new EasyPoint(poses.Observations.Grabs.Red, new Offsets().addY(vars.Mechanisms.Grabber.AtObservation.OUT)), poses.Observations.Grabs.Red.getHeading());
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
                primaryArm.setPosition(PrimaryArm.positionStates.RAISE_SPECIMEN, false);
                setPathState(liftSpecimen2Timeout);
                break;
            case liftSpecimen2Timeout:
                if (pathTimer.getElapsedTime() > 250 && primaryArm.isStopped()) {
                    setPathState(observationRetreat2);
                }
                break;
            case observationRetreat2:
                follower.holdPoint(new EasyPoint(poses.Observations.Retreats.Red), poses.Observations.Retreats.Red.getHeading());
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
                primaryArm.setPosition(PrimaryArm.positionStates.ALIGN_SPECIMEN, true);
                grabber.setGrabberState(Grabber.grabberStates.OUT);
                setPathState(chambers3Heading);
                break;
            case chambers3Heading:
                if (follower.getCurrentTValue() > 0.3) {
                    ((EasySafePath) getPath(toChambers3)).setHeading(HeadingTypes.LINEAR, poses.Observations.Grabs.Red, poses.Chambers.Red);
                    setPathState(holdChambers3);
                }
                break;
            case holdChambers3:
                if (!follower.isBusy() && primaryArm.isStopped()) {
                    follower.holdPoint(new EasyPoint(poses.Chambers.Red, new Offsets().remY(vars.Chassis.FRONT_LENGTH).remY(vars.Mechanisms.Grabber.AtChambers.OUT).addX(vars.sample).addY(1.5)), poses.Chambers.Red.getHeading());
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
                if (pathTimer.getElapsedTime() > 500 && grabber.isAtTargetAngle()) {
                    primaryArm.setPosition(PrimaryArm.positionStates.HOOK_SPECIMEN, true);
                    setPathState(chambers3LowerArmTimeout);
                }
                break;
            case chambers3LowerArmTimeout:
                if (pathTimer.getElapsedTime() > 250 && primaryArm.isStopped()) {
                    setPathState(chambers3ReleaseAndBack);
                }
                break;
            case chambers3ReleaseAndBack:
                grabber.release();
                grabber.setGrabberState(Grabber.grabberStates.OUT);
                follower.holdPoint(new EasyPoint(poses.Chambers.Retreats.Red, new Offsets().remY(vars.Chassis.FRONT_LENGTH).remY(vars.Mechanisms.Grabber.AtChambers.OUT)), poses.Chambers.Retreats.Red.getHeading());
                setPathState(chambers3ReleaseAndBackTimeout);
                break;
            case chambers3ReleaseAndBackTimeout:
                if (pathTimer.getElapsedTime() > 500) {
                    totalTime = overallTimer.getElapsedTime();
//                    requestOpModeStop();
                }
                break;
        }
    }

    public void setPathState(Red_1_2_PathStates state) {
        if (pathState != state) {
            pathState = state;
            pathTimer.resetTimer();
            autonomousPathUpdate();
        }
    }
}
