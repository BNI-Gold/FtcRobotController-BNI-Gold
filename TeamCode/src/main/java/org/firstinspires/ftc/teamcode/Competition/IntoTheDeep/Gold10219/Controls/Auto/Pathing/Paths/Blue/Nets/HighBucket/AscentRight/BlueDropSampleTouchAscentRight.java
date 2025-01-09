package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Nets.HighBucket.AscentRight;

import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Nets.HighBucket.AscentRight.PathStates.ascentRExtendArm;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Nets.HighBucket.AscentRight.PathStates.ascentRExtendArmTimeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Nets.HighBucket.AscentRight.PathStates.ascentRGrabberOut;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Nets.HighBucket.AscentRight.PathStates.ascentRLowerArm;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Nets.HighBucket.AscentRight.PathStates.ascentRTimeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Nets.HighBucket.AscentRight.PathStates.chambers1RaiseArm;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Nets.HighBucket.AscentRight.PathStates.chambers1RaiseArmTimeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Nets.HighBucket.AscentRight.PathStates.holdAscentR;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Nets.HighBucket.AscentRight.PathStates.holdNets1;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Nets.HighBucket.AscentRight.PathStates.nets1DropSample;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Nets.HighBucket.AscentRight.PathStates.nets1DropSampleTimeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Nets.HighBucket.AscentRight.PathStates.nets1ExtendArm;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Nets.HighBucket.AscentRight.PathStates.nets1ExtendArmTimeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Nets.HighBucket.AscentRight.PathStates.nets1Heading;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Nets.HighBucket.AscentRight.PathStates.nets1LowerArm;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Nets.HighBucket.AscentRight.PathStates.nets1LowerArmTimeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Nets.HighBucket.AscentRight.PathStates.nets1ResetMechanisms;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Nets.HighBucket.AscentRight.PathStates.nets1ResetMechanismsTimeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Nets.HighBucket.AscentRight.PathStates.nets1Timeout;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Nets.HighBucket.AscentRight.PathStates.opModeStop;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Nets.HighBucket.AscentRight.PathStates.toAscentR;
import static org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Blue.Nets.HighBucket.AscentRight.PathStates.toNets1;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto.Pathing.Vars.FieldPoses;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Mechanisms.Grabber.Grabber;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Mechanisms.PrimaryArm.PrimaryArm;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Robots.CompBot.CompBot;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Robots.CompBot.CompBotVars;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Utils.EasyPoint;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Utils.O;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Utils.Offsets;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Utils.Paths.EasySafePath;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Utils.Paths.HeadingTypes;

import java.util.HashMap;
import java.util.Map;

@Disabled
@Autonomous(name = "Blue Drop Sample High Bucket Touch Ascent Right", group = "Auto - Blue - HighBucket - Sample")
public class BlueDropSampleTouchAscentRight extends OpMode {
    private final CompBot Bot = new CompBot();
    private final CompBotVars vars = new CompBotVars();



    private final FieldPoses poses = new FieldPoses();

    Grabber grabber = new Grabber();

    PrimaryArm arm = new PrimaryArm();

    private Pose startPose;

    private Follower follower;

    private Timer pathTimer;

    private final Map<PathStates, Path> paths = new HashMap<>();
    private PathStates pathState;
    
    @Override
    public void init() {
        Bot.initRobot(hardwareMap);



        pathTimer = new Timer();



startPose = new Pose(96, 136, 0);

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
        setPathState(chambers1RaiseArm);
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
        telemetry.update();
    }

    private Path getPath(PathStates state) {
        return O.req(paths.get(state));
    }

    public void buildPaths() {
        paths.put(toNets1,
                new EasySafePath(startPose, poses.Nets.Blue,
                        new Offsets().remY(vars.Chassis.FRONT_LENGTH).remX(vars.Chassis.FRONT_LENGTH).remY(vars.Mechanisms.Grabber.AtChambers.OUT).remX(vars.Mechanisms.Grabber.AtChambers.OUT))
                        .setHeading(HeadingTypes.CONSTANT, startPose));

        paths.put(toAscentR,
                new EasySafePath(getPath(toNets1).getLastControlPoint(), poses.Ascents.Pre.Blue, poses.Ascents.Post.Blue, poses.Ascents.Blue.Right,
                        new Offsets().addX(vars.Chassis.FRONT_LENGTH))
                        .setHeading(HeadingTypes.LINEAR, getPath(toNets1), poses.Ascents.Blue.Right, .35));
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case chambers1RaiseArm:
                arm.setRotation(PrimaryArm.rotationStates.UP, 6, true);
                grabber.setGrabberState(Grabber.grabberStates.OUT);
                setPathState(chambers1RaiseArmTimeout);
                break;
            case chambers1RaiseArmTimeout:
                if (pathTimer.getElapsedTime() > 1000) {
                    setPathState(toNets1);
                }
                break;
            case toNets1:
                follower.followPath(getPath(toNets1));
                setPathState(nets1Heading);
                break;
            case nets1Heading:
                if (follower.getCurrentTValue() > 0.1) {
                    ((EasySafePath) getPath(toNets1)).setHeading(HeadingTypes.LINEAR, startPose, poses.Nets.Blue);
                    setPathState(holdNets1);
                }
                break;
            case holdNets1:
                if (!follower.isBusy()) {
                    follower.holdPoint(
                            new EasyPoint(poses.Nets.Blue,
                                    new Offsets().remY(vars.Chassis.FRONT_LENGTH).remX(vars.Chassis.FRONT_LENGTH).remY(vars.Mechanisms.Grabber.AtChambers.OUT).remX(vars.Mechanisms.Grabber.AtChambers.OUT)),
                            poses.Nets.Blue.getHeading()
                    );
                    setPathState(nets1Timeout);
                }
                break;
            case nets1Timeout:
                if (pathTimer.getElapsedTime() > 500 && !follower.isBusy()) {
                    setPathState(nets1ExtendArm);
                }
                break;
            case nets1ExtendArm:
                arm.setExtend();
                setPathState(nets1ExtendArmTimeout);
                break;
            case nets1ExtendArmTimeout:
                if (pathTimer.getElapsedTime() > 500) {
                    setPathState(nets1LowerArm);
                }
                break;
            case nets1LowerArm:
                arm.setRotation(PrimaryArm.rotationStates.DOWN, 1, false);
                setPathState(nets1LowerArmTimeout);
                break;
            case nets1LowerArmTimeout:
                if (pathTimer.getElapsedTime() > 500) {
                    setPathState(nets1DropSample);
                }
                break;
            case nets1DropSample:
                grabber.setGrabberState(Grabber.grabberStates.OUT);
                grabber.release();
                setPathState(nets1DropSampleTimeout);
                break;
            case nets1DropSampleTimeout:
                if (pathTimer.getElapsedTime() > 1000) {
                    setPathState(nets1ResetMechanisms);
                }
                break;
            case nets1ResetMechanisms:
                grabber.grab();
                grabber.setGrabberState(Grabber.grabberStates.OUT);
                arm.setRetract();
                setPathState(nets1ResetMechanismsTimeout);
            case nets1ResetMechanismsTimeout:
                if (pathTimer.getElapsedTime() > 1000) {
                    setPathState(toAscentR);
                }
            case toAscentR:
                follower.followPath(getPath(toAscentR));
                setPathState(holdAscentR);
                break;
            case holdAscentR:
                if (!follower.isBusy()) {
                    follower.holdPoint(
                            new EasyPoint(poses.Ascents.Blue.Right,
                                    new Offsets().addX(vars.Chassis.FRONT_LENGTH)),
                            poses.Ascents.Blue.Right.getHeading()
                    );
                    setPathState(ascentRGrabberOut);
                }
                break;
            case ascentRGrabberOut:
                grabber.setGrabberState(Grabber.grabberStates.OUT);
                setPathState(ascentRTimeout);
                break;
            case ascentRTimeout:
                if (pathTimer.getElapsedTime() > 500 && !follower.isBusy()) {
                    setPathState(ascentRExtendArm);
                }
                break;
            case ascentRExtendArm:
                arm.setExtend();
                setPathState(ascentRExtendArmTimeout);
                break;
            case ascentRExtendArmTimeout:
                if (pathTimer.getElapsedTime() > 500) {
                    setPathState(ascentRLowerArm);
                }
                break;
            case ascentRLowerArm:
                arm.setRotation(PrimaryArm.rotationStates.DOWN, 1.5, false);
                setPathState(opModeStop);
                break;
            case opModeStop:
                if (arm.getRotationState() == PrimaryArm.rotationStates.STOPPED) {
                    requestOpModeStop();
                }
                break;
        }
    }

    public void setPathState(PathStates state) {
        pathState = state;
        pathTimer.resetTimer();
        autonomousPathUpdate();
    }
}
