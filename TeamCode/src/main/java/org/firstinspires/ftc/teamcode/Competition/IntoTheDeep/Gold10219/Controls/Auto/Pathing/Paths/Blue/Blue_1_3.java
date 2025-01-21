package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue;

import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.approachGrabSpecimen1;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.approachGrabSpecimen1Timeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.chambers1Heading;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.chambers1LowerArm;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.chambers1LowerArmTimeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.chambers1RaiseArm;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.chambers1ReleaseAndBack;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.chambers1ReleaseAndBackTimeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.chambers1Timeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.chambers2Heading;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.chambers2LowerArm;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.chambers2LowerArmTimeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.chambers2RaiseArm;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.chambers2ReleaseAndBack;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.chambers2ReleaseAndBackTimeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.chambers2Timeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.chambers3Heading;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.chambers3LowerArm;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.chambers3LowerArmTimeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.chambers3RaiseArm;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.chambers3ReleaseAndBack;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.chambers3ReleaseAndBackTimeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.chambers3Timeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.chambers4Heading;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.chambers4LowerArm;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.chambers4LowerArmTimeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.chambers4RaiseArm;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.chambers4ReleaseAndBack;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.chambers4ReleaseAndBackTimeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.chambers4Timeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.grabSpecimen1;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.grabSpecimen1Timeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.grabSpecimen2;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.grabSpecimen2Timeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.grabSpecimen3;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.grabSpecimen3Timeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.holdChambers1;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.holdChambers2;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.holdChambers3;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.holdChambers4;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.holdObservation1;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.holdObservation1Timeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.holdObservation2;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.holdObservation2Timeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.holdObservation3;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.holdObservation3Timeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.holdObservation4;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.holdObservation4Timeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.holdObservation5;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.holdObservation5Timeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.holdSample1;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.holdSample2;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.liftSpecimen1;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.liftSpecimen1Timeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.liftSpecimen2;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.liftSpecimen2Timeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.liftSpecimen3;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.liftSpecimen3Timeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.observationRetreat1;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.observationRetreat1Timeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.observationRetreat2;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.observationRetreat2Timeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.observationRetreat3;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.observationRetreat3Timeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.toChambers1;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.toChambers2;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.toChambers3;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.toChambers4;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.toObservation1;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.toObservation2;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.toObservation3;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.toObservation4;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.toObservation5;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.toSample1;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Blue_1_3_PathStates.toSample2;

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

@Autonomous(name = "A - Blue 1 + 3", group = "Auto - Blue")
public class Blue_1_3 extends OpMode {
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

    private final Map<Blue_1_3_PathStates, Path> paths = new HashMap<>();
    private Blue_1_3_PathStates pathState;
    private Blue_1_3_PathStates savedPathState;

    double pushSampleOffset = 2;

    @Override
    public void init() {
        Bot.initRobot(hardwareMap);

        pathTimer = new Timer();
        overallTimer = new Timer();

        startPose = new Pose(48, 136, Math.toRadians(0));

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
        overallTimer.resetTimer();
    }

    double totalTime = 0;

    public void loop() {
        follower.update();
        for (int i = 0; i < 3; i++) {
            grabber.tiltStateCheck();
        }
        primaryArm.positionChecker();
        if (totalTime == 0.0) {
            if ((grabber.imu.getSystemStatus() == BNO055IMU.SystemStatus.SYSTEM_ERROR || grabber.imu.getSystemStatus() == BNO055IMU.SystemStatus.IDLE) && pathState != Blue_1_3_PathStates.STOP_FOR_IMU_RESET) {
                savedPathState = pathState;               // remember where you were
                setPathState(Blue_1_3_PathStates.STOP_FOR_IMU_RESET);
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

    private Path getPath(Blue_1_3_PathStates state) {
        return O.req(paths.get(state));
    }

    public void buildPaths() {
        paths.put(toChambers1, new EasySafePath(startPose, poses.Chambers.Blue, new Offsets().addY(vars.Chassis.FRONT_LENGTH).addY(vars.Mechanisms.Grabber.AtChambers.OUT).addX(vars.sample)).setHeading(HeadingTypes.CONSTANT, startPose));

        paths.put(toSample1, new EasySafePath(getPath(toChambers1).getLastControlPoint(), poses.SampleLines.Blue.B1.Slip, poses.SampleLines.Blue.B1.Post, poses.SampleLines.Blue.B1.Sample, new Offsets().remY(vars.Chassis.FRONT_LENGTH)).setHeading(HeadingTypes.LINEAR, getPath(toChambers1), poses.SampleLines.Blue.pushApproachAngle, .35));

        paths.put(toObservation1, new EasySafePath(getPath(toSample1).getLastControlPoint(), poses.SampleLines.Blue.B1.Pre, poses.Observations.Grabs.Blue, new Offsets().remY(vars.Mechanisms.Grabber.AtObservation.OUT).addY(pushSampleOffset).remX(pushSampleOffset)).setHeading(HeadingTypes.CONSTANT, poses.Observations.Grabs.Blue));

        paths.put(toSample2, new EasySafePath(getPath(toObservation1).getLastControlPoint(), poses.SampleLines.Blue.B2.Slip, poses.SampleLines.Blue.B2.Post, poses.SampleLines.Blue.B2.Sample, new Offsets().remY(vars.Chassis.FRONT_LENGTH)).setHeading(HeadingTypes.CONSTANT, poses.Observations.Grabs.Blue));

        paths.put(toObservation2, new EasySafePath(getPath(toSample2).getLastControlPoint(), poses.Observations.Grabs.Blue, new Offsets().remY(vars.Mechanisms.Grabber.AtObservation.OUT).addY(pushSampleOffset).remX(pushSampleOffset)).setHeading(HeadingTypes.CONSTANT, poses.Observations.Grabs.Blue));

        paths.put(toChambers2, new EasySafePath(getPath(toObservation2).getLastControlPoint(), poses.Observations.Retreats.Blue, poses.Chambers.Midpoints.Blue, poses.Chambers.Blue, new Offsets().addY(vars.Chassis.FRONT_LENGTH).addY(vars.Mechanisms.Grabber.AtChambers.OUT)).setHeading(HeadingTypes.CONSTANT, poses.Observations.Grabs.Blue));

        paths.put(toObservation3, new EasySafePath(getPath(toChambers2).getLastControlPoint(), poses.Observations.Grabs.Blue, new Offsets().remY(vars.Mechanisms.Grabber.AtObservation.OUT)).setHeading(HeadingTypes.CONSTANT, poses.Observations.Grabs.Blue));

        paths.put(toChambers3, new EasySafePath(getPath(toObservation3).getLastControlPoint(), poses.Observations.Retreats.Blue, poses.Chambers.Midpoints.Blue, poses.Chambers.Blue, new Offsets().addY(vars.Chassis.FRONT_LENGTH).addY(vars.Mechanisms.Grabber.AtChambers.OUT).remX(vars.sample)).setHeading(HeadingTypes.CONSTANT, poses.Observations.Grabs.Blue));

        paths.put(toObservation4, new EasySafePath(getPath(toChambers3).getLastControlPoint(), poses.Observations.Grabs.Blue, new Offsets().remY(vars.Mechanisms.Grabber.AtObservation.OUT)).setHeading(HeadingTypes.CONSTANT, poses.Observations.Grabs.Blue));

        paths.put(toChambers4, new EasySafePath(getPath(toObservation4).getLastControlPoint(), poses.Observations.Retreats.Blue, poses.Chambers.Midpoints.Blue, poses.Chambers.Blue, new Offsets().addY(vars.Chassis.FRONT_LENGTH).addY(vars.Mechanisms.Grabber.AtChambers.OUT).remX(vars.sample).remX(vars.sample)).setHeading(HeadingTypes.CONSTANT, poses.Observations.Grabs.Blue));

        paths.put(toObservation5, new EasySafePath(getPath(toChambers4).getLastControlPoint(), poses.Observations.Grabs.Blue, new Offsets().remY(vars.Mechanisms.Grabber.AtObservation.OUT).addY(pushSampleOffset).remX(pushSampleOffset).addX(10)).setHeading(HeadingTypes.CONSTANT, Math.toRadians(-90)));
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
                setPathState(chambers1Heading);
                break;
            case chambers1Heading:
                if (follower.getCurrentTValue() > 0.1) {
                    ((EasySafePath) getPath(toChambers1)).setHeading(HeadingTypes.LINEAR, startPose, poses.Chambers.Blue);
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
                if (!follower.isBusy() && primaryArm.isAtPosition() && grabber.isAtTargetAngle()) {
                    follower.holdPoint(new EasyPoint(poses.Chambers.Blue, new Offsets().addY(vars.Chassis.FRONT_LENGTH).addY(vars.Mechanisms.Grabber.AtChambers.OUT).addX(vars.sample)), poses.Chambers.Blue.getHeading());
                    setPathState(chambers1Timeout);
                }
                break;
            case chambers1Timeout:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 250 && grabber.isAtTargetAngle()) {
                    setPathState(chambers1LowerArm);
                }
                break;
            case chambers1LowerArm:
                grabber.setGrabberState(Grabber.grabberStates.HOOK);
                if (grabber.isAtTargetAngle()) {
                    primaryArm.setPosition(PrimaryArm.positionStates.HOOK_SPECIMEN, true);
                    setPathState(chambers1LowerArmTimeout);
                }
                break;
            case chambers1LowerArmTimeout:
                if (primaryArm.isAtPosition() && pathTimer.getElapsedTime() > 500) {
                    setPathState(chambers1ReleaseAndBack);
                }
                break;
            case chambers1ReleaseAndBack:
                grabber.release();
                grabber.setGrabberState(Grabber.grabberStates.OUT);
                follower.holdPoint(new EasyPoint(poses.Chambers.Retreats.Blue, new Offsets().addY(vars.Chassis.FRONT_LENGTH).addY(vars.Mechanisms.Grabber.AtChambers.OUT)), poses.Chambers.Retreats.Blue.getHeading());
                grabber.setGrabberState(Grabber.grabberStates.DOWN);
                primaryArm.setPosition(PrimaryArm.positionStates.ALIGN_SPECIMEN, true);
                setPathState(chambers1ReleaseAndBackTimeout);
                break;
            case chambers1ReleaseAndBackTimeout:
                if (pathTimer.getElapsedTime() > 250) {
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
                    follower.holdPoint(new EasyPoint(poses.SampleLines.Blue.B1.Sample), poses.SampleLines.Blue.pushApproachAngle);
                    setPathState(toObservation1);
                }
                break;
            case toObservation1:
                follower.followPath(getPath(toObservation1));
                setPathState(holdObservation1);
                break;
            case holdObservation1:
                if (!follower.isBusy()) {
                    follower.holdPoint(new EasyPoint(poses.Observations.Grabs.Blue, new Offsets().remY(vars.Mechanisms.Grabber.AtObservation.OUT).addY(pushSampleOffset).remX(pushSampleOffset)), poses.Observations.Grabs.Blue.getHeading());

                    setPathState(holdObservation1Timeout);
                }
                break;
            case holdObservation1Timeout:
                if (pathTimer.getElapsedTime() > 250 && !follower.isBusy()) {
                    setPathState(toSample2);
                }
            case toSample2:
                follower.followPath(getPath(toSample2));
                setPathState(holdSample2);
                break;
            case holdSample2:
                if (!follower.isBusy()) {
                    grabber.setGrabberState(Grabber.grabberStates.DOWN);
                    follower.holdPoint(new EasyPoint(poses.SampleLines.Blue.B2.Sample), poses.SampleLines.Blue.pushApproachAngle);
                    setPathState(toObservation2);
                }
                break;
            case toObservation2:
                follower.followPath(getPath(toObservation2));
                primaryArm.setPosition(PrimaryArm.positionStates.GRAB_SPECIMEN, true);
                grabber.setGrabberState(Grabber.grabberStates.OUT);
                setPathState(holdObservation2);
                break;
            case holdObservation2:
                if (!follower.isBusy()) {
                    follower.holdPoint(new EasyPoint(poses.Observations.Grabs.Blue, new Offsets().remY(vars.Mechanisms.Grabber.AtObservation.OUT).addY(pushSampleOffset).remX(pushSampleOffset)), poses.Observations.Grabs.Blue.getHeading());
                    setPathState(holdObservation2Timeout);
                }
                break;
            case holdObservation2Timeout:
                if (pathTimer.getElapsedTime() > 250 && !follower.isBusy()) {
                    setPathState(approachGrabSpecimen1);
                }
                break;
            case approachGrabSpecimen1:
                follower.holdPoint(new EasyPoint(poses.Observations.Grabs.Blue, new Offsets().remY(vars.Mechanisms.Grabber.AtObservation.OUT)), poses.Observations.Grabs.Blue.getHeading());
                setPathState(approachGrabSpecimen1Timeout);
                break;
            case approachGrabSpecimen1Timeout:
                if (pathTimer.getElapsedTime() > 500 && !follower.isBusy())
                    setPathState(grabSpecimen1);
                break;
            case grabSpecimen1:
                grabber.grab();
                setPathState(grabSpecimen1Timeout);
                break;
            case grabSpecimen1Timeout:
                if (pathTimer.getElapsedTime() > 250) {
                    setPathState(liftSpecimen1);
                }
                break;
            case liftSpecimen1:
                primaryArm.setPosition(PrimaryArm.positionStates.RAISE_SPECIMEN, false);
                setPathState(liftSpecimen1Timeout);
                break;
            case liftSpecimen1Timeout:
                if (pathTimer.getElapsedTime() > 250 && primaryArm.isAtPosition()) {
                    setPathState(observationRetreat1);
                }
                break;
            case observationRetreat1:
                follower.holdPoint(new EasyPoint(poses.Observations.Retreats.Blue), poses.Observations.Retreats.Blue.getHeading());
                setPathState(observationRetreat1Timeout);
                break;
            case observationRetreat1Timeout:
                if (pathTimer.getElapsedTime() > 250 && !follower.isBusy()) {
                    setPathState(toChambers2);
                }
                break;
            case toChambers2:
                follower.followPath(getPath(toChambers2));
                setPathState(chambers2Heading);
                break;
            case chambers2Heading:
                if (follower.getCurrentTValue() > 0.1) {
                    ((EasySafePath) getPath(toChambers2)).setHeading(HeadingTypes.LINEAR, poses.Observations.Grabs.Blue, poses.Chambers.Blue);
                    setPathState(chambers2RaiseArm);
                }
                break;
            case chambers2RaiseArm:
                primaryArm.setPosition(PrimaryArm.positionStates.ALIGN_SPECIMEN, true);
                grabber.setGrabberState(Grabber.grabberStates.OUT);
                setPathState(holdChambers2);
                break;
            case holdChambers2:
                if (!follower.isBusy() && primaryArm.isAtPosition() && grabber.isAtTargetAngle()) {
                    follower.holdPoint(new EasyPoint(poses.Chambers.Blue, new Offsets().addY(vars.Chassis.FRONT_LENGTH).addY(vars.Mechanisms.Grabber.AtChambers.OUT)), poses.Chambers.Blue.getHeading());
                    setPathState(chambers2Timeout);
                }
                break;
            case chambers2Timeout:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 250 && grabber.isAtTargetAngle()) {
                    setPathState(chambers2LowerArm);
                }
                break;
            case chambers2LowerArm:
                grabber.setGrabberState(Grabber.grabberStates.HOOK);
                if (grabber.isAtTargetAngle()) {
                    primaryArm.setPosition(PrimaryArm.positionStates.HOOK_SPECIMEN, true);
                    setPathState(chambers2LowerArmTimeout);
                }
                break;
            case chambers2LowerArmTimeout:
                if (primaryArm.isAtPosition() && pathTimer.getElapsedTime() > 500) {
                    setPathState(chambers2ReleaseAndBack);
                }
                break;
            case chambers2ReleaseAndBack:
                grabber.release();
                grabber.setGrabberState(Grabber.grabberStates.OUT);
                follower.holdPoint(new EasyPoint(poses.Chambers.Retreats.Blue, new Offsets().addY(vars.Chassis.FRONT_LENGTH).addY(vars.Mechanisms.Grabber.AtChambers.OUT)), poses.Chambers.Retreats.Blue.getHeading());
                grabber.setGrabberState(Grabber.grabberStates.DOWN);
                primaryArm.setPosition(PrimaryArm.positionStates.ALIGN_SPECIMEN, true);
                setPathState(chambers2ReleaseAndBackTimeout);
                break;
            case chambers2ReleaseAndBackTimeout:
                if (pathTimer.getElapsedTime() > 250) {
                    setPathState(toObservation3);
                }
                break;
            case toObservation3:
                follower.followPath(getPath(toObservation3));
                primaryArm.setPosition(PrimaryArm.positionStates.GRAB_SPECIMEN, true);
                grabber.setGrabberState(Grabber.grabberStates.OUT);
                setPathState(holdObservation3);
                break;
            case holdObservation3:
                if (!follower.isBusy()) {
                    follower.holdPoint(new EasyPoint(poses.Observations.Grabs.Blue, new Offsets().remY(vars.Mechanisms.Grabber.AtObservation.OUT).remY(.5)), poses.Observations.Grabs.Blue.getHeading());
                    setPathState(holdObservation3Timeout);
                }
                break;
            case holdObservation3Timeout:
                if (pathTimer.getElapsedTime() > 500 && !follower.isBusy()) {
                    setPathState(grabSpecimen2);
                }
                break;
            case grabSpecimen2:
                grabber.grab();
                setPathState(grabSpecimen2Timeout);
                break;
            case grabSpecimen2Timeout:
                if (pathTimer.getElapsedTime() > 250) {
                    setPathState(liftSpecimen2);
                }
                break;
            case liftSpecimen2:
                primaryArm.setPosition(PrimaryArm.positionStates.RAISE_SPECIMEN, false);
                setPathState(liftSpecimen2Timeout);
                break;
            case liftSpecimen2Timeout:
                if (pathTimer.getElapsedTime() > 250 && primaryArm.isAtPosition()) {
                    setPathState(observationRetreat2);
                }
                break;
            case observationRetreat2:
                follower.holdPoint(new EasyPoint(poses.Observations.Retreats.Blue), poses.Observations.Retreats.Blue.getHeading());
                setPathState(observationRetreat2Timeout);
                break;
            case observationRetreat2Timeout:
                if (pathTimer.getElapsedTime() > 250 && !follower.isBusy()) {
                    setPathState(toChambers3);
                }
                break;
            case toChambers3:
                follower.followPath(getPath(toChambers3));
                setPathState(chambers3Heading);
                break;
            case chambers3Heading:
                if (follower.getCurrentTValue() > 0.1) {
                    ((EasySafePath) getPath(toChambers3)).setHeading(HeadingTypes.LINEAR, poses.Observations.Grabs.Blue, poses.Chambers.Blue);
                    setPathState(chambers3RaiseArm);
                }
                break;
            case chambers3RaiseArm:
                primaryArm.setPosition(PrimaryArm.positionStates.ALIGN_SPECIMEN, true);
                grabber.setGrabberState(Grabber.grabberStates.OUT);
                setPathState(holdChambers3);
                break;
            case holdChambers3:
                if (!follower.isBusy() && primaryArm.isAtPosition() && grabber.isAtTargetAngle()) {
                    follower.holdPoint(new EasyPoint(poses.Chambers.Blue, new Offsets().addY(vars.Chassis.FRONT_LENGTH).addY(vars.Mechanisms.Grabber.AtChambers.OUT).remX(vars.sample).remY(1.5)), poses.Chambers.Blue.getHeading());
                    setPathState(chambers3Timeout);
                }
                break;
            case chambers3Timeout:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 250 && grabber.isAtTargetAngle()) {
                    setPathState(chambers3LowerArm);
                }
                break;

            case chambers3LowerArm:
                grabber.setGrabberState(Grabber.grabberStates.HOOK);
                if (grabber.isAtTargetAngle()) {
                    primaryArm.setPosition(PrimaryArm.positionStates.HOOK_SPECIMEN, true);
                    setPathState(chambers3LowerArmTimeout);
                }
                break;
            case chambers3LowerArmTimeout:
                if (primaryArm.isAtPosition() && pathTimer.getElapsedTime() > 500) {
                    setPathState(chambers3ReleaseAndBack);
                }
                break;
            case chambers3ReleaseAndBack:
                grabber.release();
                grabber.setGrabberState(Grabber.grabberStates.OUT);
                follower.holdPoint(new EasyPoint(poses.Chambers.Retreats.Blue, new Offsets().addY(vars.Chassis.FRONT_LENGTH).addY(vars.Mechanisms.Grabber.AtChambers.OUT)), poses.Chambers.Retreats.Blue.getHeading());
                grabber.setGrabberState(Grabber.grabberStates.DOWN);
                primaryArm.setPosition(PrimaryArm.positionStates.ALIGN_SPECIMEN, true);
                setPathState(chambers3ReleaseAndBackTimeout);
                break;
            case chambers3ReleaseAndBackTimeout:
                if (pathTimer.getElapsedTime() > 250) {
                    setPathState(toObservation4);
                }
                break;
            case toObservation4:
                follower.followPath(getPath(toObservation4));
                primaryArm.setPosition(PrimaryArm.positionStates.GRAB_SPECIMEN, true);
                grabber.setGrabberState(Grabber.grabberStates.OUT);
                setPathState(holdObservation4);
                break;
            case holdObservation4:
                if (!follower.isBusy()) {
                    follower.holdPoint(new EasyPoint(poses.Observations.Grabs.Blue, new Offsets().remY(vars.Mechanisms.Grabber.AtObservation.OUT).remY(.5)), poses.Observations.Grabs.Blue.getHeading());
                    setPathState(holdObservation4Timeout);
                }
                break;
            case holdObservation4Timeout:
                if (pathTimer.getElapsedTime() > 500 && !follower.isBusy()) {
                    setPathState(grabSpecimen3);
                }
                break;
            case grabSpecimen3:
                grabber.grab();
                setPathState(grabSpecimen3Timeout);
                break;
            case grabSpecimen3Timeout:
                if (pathTimer.getElapsedTime() > 250) {
                    setPathState(liftSpecimen3);
                }
                break;
            case liftSpecimen3:
                primaryArm.setPosition(PrimaryArm.positionStates.RAISE_SPECIMEN, false);
                setPathState(liftSpecimen3Timeout);
                break;
            case liftSpecimen3Timeout:
                if (pathTimer.getElapsedTime() > 250 && primaryArm.isAtPosition()) {
                    setPathState(observationRetreat3);
                }
                break;
            case observationRetreat3:
                follower.holdPoint(new EasyPoint(poses.Observations.Retreats.Blue), poses.Observations.Retreats.Blue.getHeading());
                setPathState(observationRetreat3Timeout);
                break;
            case observationRetreat3Timeout:
                if (pathTimer.getElapsedTime() > 250 && !follower.isBusy()) {
                    setPathState(toChambers4);
                }
                break;
            case toChambers4:
                follower.followPath(getPath(toChambers4));
                setPathState(chambers4Heading);
                break;
            case chambers4Heading:
                if (follower.getCurrentTValue() > 0.1) {
                    ((EasySafePath) getPath(toChambers4)).setHeading(HeadingTypes.LINEAR, poses.Observations.Grabs.Blue, poses.Chambers.Blue);
                    setPathState(chambers4RaiseArm);
                }
                break;
            case chambers4RaiseArm:
                primaryArm.setPosition(PrimaryArm.positionStates.ALIGN_SPECIMEN, true);
                grabber.setGrabberState(Grabber.grabberStates.OUT);
                setPathState(holdChambers4);
                break;
            case holdChambers4:
                if (!follower.isBusy() && primaryArm.isAtPosition() && grabber.isAtTargetAngle()) {
                    follower.holdPoint(new EasyPoint(poses.Chambers.Blue, new Offsets().addY(vars.Chassis.FRONT_LENGTH).addY(vars.Mechanisms.Grabber.AtChambers.OUT).remX(vars.sample).remX(vars.sample).remY(1.5)), poses.Chambers.Blue.getHeading());
                    setPathState(chambers4Timeout);
                }
                break;
            case chambers4Timeout:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 250 && grabber.isAtTargetAngle()) {
                    setPathState(chambers4LowerArm);
                }
                break;

            case chambers4LowerArm:
                grabber.setGrabberState(Grabber.grabberStates.HOOK);
                if (grabber.isAtTargetAngle()) {
                    primaryArm.setPosition(PrimaryArm.positionStates.HOOK_SPECIMEN, true);
                    setPathState(chambers4LowerArmTimeout);
                }
                break;
            case chambers4LowerArmTimeout:
                if (primaryArm.isAtPosition() && pathTimer.getElapsedTime() > 500) {
                    setPathState(chambers4ReleaseAndBack);
                }
                break;
            case chambers4ReleaseAndBack:
                grabber.release();
                grabber.setGrabberState(Grabber.grabberStates.OUT);
                follower.holdPoint(new EasyPoint(poses.Chambers.Retreats.Blue, new Offsets().addY(vars.Chassis.FRONT_LENGTH).addY(vars.Mechanisms.Grabber.AtChambers.OUT)), poses.Chambers.Retreats.Blue.getHeading());
                grabber.setGrabberState(Grabber.grabberStates.DOWN);
                primaryArm.setPosition(PrimaryArm.positionStates.ALIGN_SPECIMEN, true);
                setPathState(chambers4ReleaseAndBackTimeout);
                break;
            case chambers4ReleaseAndBackTimeout:
                if (pathTimer.getElapsedTime() > 250) {
                    setPathState(toObservation5);
                }
                break;
            case toObservation5:
                follower.followPath(getPath(toObservation5));
                setPathState(holdObservation5);
                break;
            case holdObservation5:
                if (!follower.isBusy()) {
                    follower.holdPoint(new EasyPoint(poses.Observations.Grabs.Blue, new Offsets().remY(vars.Mechanisms.Grabber.AtObservation.OUT).addY(pushSampleOffset).remX(pushSampleOffset).addX(10)), Math.toRadians(-90));
                    setPathState(holdObservation5Timeout);
                }
                break;
            case holdObservation5Timeout:
                if (pathTimer.getElapsedTime() > 250 && !follower.isBusy()) {
                    totalTime = overallTimer.getElapsedTime();
//                    requestOpModeStop();
                }
                break;
        }
    }

    public void setPathState(Blue_1_3_PathStates state) {
        if (pathState != state) {
            pathState = state;
            pathTimer.resetTimer();
            autonomousPathUpdate();
        }
    }
}
