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

    Pose2D nP = null;
    Pose nP2 = null;
    Pose a = null;

    public void tel() {
        Pose2D current = pose.getPose();
        telemetry.addData("PX: ", current.getX(DistanceUnit.INCH));
        telemetry.addData("PY: ", current.getY(DistanceUnit.INCH));
        telemetry.addData("PO: ", current.getHeading(AngleUnit.DEGREES));
        telemetry.addLine();
        telemetry.addData("pathState: ", pathState);
//        if (nP != null) {
//            telemetry.addLine();
//            telemetry.addData("New Pose X: ", nP.getX(DistanceUnit.INCH));
//            telemetry.addData("New Pose Y: ", nP.getY(DistanceUnit.INCH));
//            telemetry.addData("New Pose H: ", nP.getHeading(AngleUnit.DEGREES));
//        }
        if (a != null) {
            telemetry.addLine();
            telemetry.addData("A X: ", a.getX());
            telemetry.addData("A Y: ", a.getY());
            telemetry.addData("A H: ", a.getHeading());
        }
        if (nP2 != null) {
            telemetry.addLine();
            telemetry.addData("NP2 X: ", nP2.getX());
            telemetry.addData("NP2 Y: ", nP2.getY());
            telemetry.addData("NP2 H: ", nP2.getHeading());
        }
        telemetry.update();
    }

    public void buildPaths() {
        fromStartToChambers = new EasyPath(startPose, poses.Chambers.Blue, new double[]{}, new double[]{vars.Chassis.FRONT_LENGTH + vars.Mechanisms.Grabber.GRABBER_HOOK_POSITION});
        fromStartToChambers.setConstantHeadingInterpolation(startPose.getHeading());
        fromStartToChambers.setPathEndTimeoutConstraint(3);

        fromChambersToRecal = new EasyPath(fromStartToChambers.getLastControlPoint(), poses.Recalibration.Single.A11);
        fromChambersToRecal.setLinearHeadingInterpolation(new SafeInterpolationStartHeading(fromStartToChambers.getEndTangent().getTheta(), poses.Recalibration.Single.A11).getValue(), poses.Recalibration.Single.A11.getHeading(), .8);
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
                    follower.holdPoint(new BezierPoint(fromChambersToRecal.getLastControlPoint()), Math.toRadians(-180));

                    if (Math.abs(pose.getPose().getHeading(AngleUnit.DEGREES) - -180) < 2) {
                        setPathState(16);
                    }
                }
                break;
            case 16:
                pose.syncPose();
                pose.updatePose();

                Pose2D pPose = pose.getPose();
                nP = pPose;
                //-44, 120, -178
//                Pose newPose = new Pose(Math.abs(pPose.getX(DistanceUnit.INCH)), pPose.getY(DistanceUnit.INCH), pPose.getHeading(AngleUnit.RADIANS));
                Pose newPose = new Pose(48, 120, Math.toRadians(-180));
                nP2 = newPose;

                follower.setCurrentPoseWithOffset(newPose);

                a = follower.getPose();

                setPathState(17);
                break;
            case 17:
                // Align the robot to the start tangent of the path
                follower.holdPoint(new BezierPoint(fromRecalToObservation.getFirstControlPoint()), fromRecalToObservation.getFirstControlPoint().getTheta());

                if (Math.abs(pose.getPose().getHeading(AngleUnit.DEGREES) - Math.toDegrees(fromRecalToObservation.getFirstControlPoint().getTheta())) < 2) {
                    follower.followPath(fromRecalToObservation);
                    setPathState(18);
                }
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
