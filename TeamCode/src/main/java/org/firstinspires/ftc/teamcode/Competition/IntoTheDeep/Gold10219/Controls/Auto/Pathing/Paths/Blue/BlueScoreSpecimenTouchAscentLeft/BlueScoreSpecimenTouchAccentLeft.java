package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.BlueScoreSpecimenTouchAscentLeft;

import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.BlueScoreSpecimenTouchAscentLeft.PathStates.ascentLExtendArm;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.BlueScoreSpecimenTouchAscentLeft.PathStates.ascentLExtendArmTimeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.BlueScoreSpecimenTouchAscentLeft.PathStates.ascentLLowerArm;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.BlueScoreSpecimenTouchAscentLeft.PathStates.ascentLGrabberOut;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.BlueScoreSpecimenTouchAscentLeft.PathStates.ascentLTimeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.BlueScoreSpecimenTouchAscentLeft.PathStates.chambers1Heading;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.BlueScoreSpecimenTouchAscentLeft.PathStates.chambers1LowerArm;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.BlueScoreSpecimenTouchAscentLeft.PathStates.chambers1LowerArmTimeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.BlueScoreSpecimenTouchAscentLeft.PathStates.chambers1RaiseArm;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.BlueScoreSpecimenTouchAscentLeft.PathStates.chambers1ReleaseAndBack;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.BlueScoreSpecimenTouchAscentLeft.PathStates.chambers1ReleaseAndBackTimeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.BlueScoreSpecimenTouchAscentLeft.PathStates.chambers1Timeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.BlueScoreSpecimenTouchAscentLeft.PathStates.holdAscentL;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.BlueScoreSpecimenTouchAscentLeft.PathStates.holdChambers1;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.BlueScoreSpecimenTouchAscentLeft.PathStates.opModeStop;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.BlueScoreSpecimenTouchAscentLeft.PathStates.toAscentL;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.BlueScoreSpecimenTouchAscentLeft.PathStates.toChambers1;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.BotPose.Pinpoint;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.BotPose.PoseHelper;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.BotPose.Vision;
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

@Autonomous(name = "Blue Score Specimen Touch Accent Left", group = "Auto - Blue")
public class BlueScoreSpecimenTouchAccentLeft extends OpMode {
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

    private final Map<PathStates, Path> paths = new HashMap<>();
    private PathStates pathState;

    private final Map<PathStates, Double> specimenOffsets = new HashMap<>();

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

    private Path getPath(PathStates state) {
        return O.req(paths.get(state));
    }

    public void buildPaths() {
        paths.put(toChambers1,
                new EasySafePath(startPose, poses.Chambers.Blue,
                        new Offsets().addY(vars.Chassis.FRONT_LENGTH).addY(vars.Mechanisms.Grabber.AtChambers.OUT))
                        .setHeading(HeadingTypes.CONSTANT, startPose));

        paths.put(toAscentL,
                new EasySafePath(getPath(toChambers1).getLastControlPoint(), poses.Ascents.Midpoints.Blue, poses.Ascents.Blue.Left,
                        new Offsets().addX(vars.Chassis.FRONT_LENGTH))
                        .setHeading(HeadingTypes.LINEAR, getPath(toChambers1), poses.Ascents.Blue.Left, .35));
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
//                arm.up(6, true);
                grabber.setGrabberState(Grabber.grabberStates.OUT);
                setPathState(holdChambers1);
                break;
            case holdChambers1:
                if (!follower.isBusy()) {
                    follower.holdPoint(
                            new EasyPoint(poses.Chambers.Blue,
                                    new Offsets().addY(vars.Chassis.FRONT_LENGTH).addY(vars.Mechanisms.Grabber.AtChambers.OUT)),
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
                    arm.setRotation(PrimaryArm.rotationStates.DOWN, .75, false);
//                    arm.down(.75, false);
                    setPathState(chambers1LowerArmTimeout);
                }
                break;
            case chambers1LowerArmTimeout:
                if (pathTimer.getElapsedTime() > 500) {
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
                    setPathState(toAscentL);
                }
                break;
            case toAscentL:
                follower.followPath(getPath(toAscentL));
                setPathState(holdAscentL);
                break;
            case holdAscentL:
                if (!follower.isBusy()) {
                    follower.holdPoint(
                            new EasyPoint(poses.Ascents.Blue.Left,
                                    new Offsets().addX(vars.Chassis.FRONT_LENGTH)),
                            poses.Ascents.Blue.Left.getHeading()
                    );
                    setPathState(ascentLGrabberOut);
                }
            case ascentLGrabberOut:
                grabber.setGrabberState(Grabber.grabberStates.OUT);
                setPathState(ascentLTimeout);
                break;
            case ascentLTimeout:
                if (pathTimer.getElapsedTime() > 500) {
                    setPathState(ascentLExtendArm);
                }
                break;
            case ascentLExtendArm:
                arm.setExtend();
                setPathState(ascentLExtendArmTimeout);
                break;
            case ascentLExtendArmTimeout:
                if (pathTimer.getElapsedTime() > 500) {
                    setPathState(ascentLLowerArm);
                }
                break;
            case ascentLLowerArm:
                arm.setRotation(PrimaryArm.rotationStates.DOWN, 1.75, false);
//                arm.down(1.75, false);
                setPathState(opModeStop);
                break;
            case opModeStop:
                requestOpModeStop();
                break;
        }
    }

    public void setPathState(PathStates state) {
        pathState = state;
        pathTimer.resetTimer();
        autonomousPathUpdate();
    }
}
