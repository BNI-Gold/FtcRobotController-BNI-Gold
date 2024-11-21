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
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.pedroPathing.util.Timer;

@Autonomous(name = "Playground 1", group = "Playground")
public class Playground1 extends OpMode {
    private CompBot Bot = new CompBot();
    private CompBotVars vars = new CompBotVars();

    private Vision vision = new Vision();
    private Pinpoint pinpoint = new Pinpoint();
    private PoseHelper pose = new PoseHelper();

    private FieldPoses poses = new FieldPoses();

    //TODO: Don't forget to set this pose on auto start when robot determines current position
    private Pose startPose;

    private Follower follower;

    private Timer pathTimer;

    private Path fromStartToChambers, fromChambersToRecal, fromRecalToObservation;
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

    public void tel() {
        Pose2D current = pose.getPose();
        telemetry.addData("PX: ", current.getX(DistanceUnit.INCH));
        telemetry.addData("PY: ", current.getY(DistanceUnit.INCH));
        telemetry.addData("PO: ", current.getHeading(AngleUnit.DEGREES));
    }

    public void buildPaths() {
        fromStartToChambers = new EasyPath(startPose, poses.Chambers.Blue, new double[]{}, new double[]{vars.Chassis.FRONT_LENGTH + vars.Mechanisms.Grabber.GRABBER_HOOK_POSITION});
        fromStartToChambers.setConstantHeadingInterpolation(startPose.getHeading());
        fromStartToChambers.setPathEndTimeoutConstraint(3);

        fromChambersToRecal = new EasyPath(fromStartToChambers.getLastControlPoint(), poses.Recalibration.A11);
        fromChambersToRecal.setLinearHeadingInterpolation(new SafeInterpolationStartHeading(fromStartToChambers.getEndTangent().getTheta(), poses.Recalibration.A11).getValue(), poses.Recalibration.A11.getHeading(), .8);
        fromChambersToRecal.setPathEndTimeoutConstraint(3);

        fromRecalToObservation = new EasyPath(fromChambersToRecal.getLastControlPoint(), poses.Observations.Blue, new double[]{}, new double[]{-vars.Chassis.FRONT_LENGTH - vars.Mechanisms.Grabber.GRABBER_EXTENDED_POSITION});
        fromRecalToObservation.setLinearHeadingInterpolation(new SafeInterpolationStartHeading(fromChambersToRecal.getEndTangent().getTheta(), poses.Observations.Blue).getValue(), poses.Observations.Blue.getHeading(), .8);
        fromRecalToObservation.setPathEndTimeoutConstraint(3);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 10:
                follower.followPath(fromStartToChambers);
                setPathState(11);
                break;
            case 11:
                if (follower.getCurrentTValue() > 0.1) {
                    fromStartToChambers.setLinearHeadingInterpolation(new SafeInterpolationStartHeading(startPose, poses.Chambers.Blue).getValue(), poses.Chambers.Blue.getHeading());
                    setPathState(12);
                }
            case 12:
                if (!follower.isBusy()) {
                    follower.holdPoint(new BezierPoint(fromStartToChambers.getLastControlPoint()), fromStartToChambers.getEndTangent().getTheta());
//                    follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), Math.toRadians(-90)));
                    setPathState(13);
                }
                break;
            case 13:
                if (pathTimer.getElapsedTime() > 10000) {
                    setPathState(14);
                }
            case 14:
                follower.followPath(fromChambersToRecal);
                setPathState(15);
                break;
            case 15:
                if (!follower.isBusy()) {
                    follower.holdPoint(new BezierPoint(fromChambersToRecal.getLastControlPoint()), fromChambersToRecal.getEndTangent().getTheta());
                    setPathState(16);
                }
                break;
            case 16:
                pose.syncPose();
                pose.updatePose();

                follower.update();
                setPathState(17);
                break;
            case 17:
                follower.followPath(fromRecalToObservation);
                setPathState(18);
                break;
            case 18:
                if (!follower.isBusy()) {
                    follower.holdPoint(new BezierPoint(fromRecalToObservation.getLastControlPoint()), Math.toRadians(90));
                }
        }
    }

    public void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
        autonomousPathUpdate();
    }
}
