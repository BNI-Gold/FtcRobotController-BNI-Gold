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

    private Path fromStartToChambers, fromChambersToObservation;
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
        fromStartToChambers = new EasyPath(startPose, poses.Chambers.Blue);
        //Commenting this to try setting linear heading interpolation after 10% of path is completed (see pathState case 11)
//        fromStartToChambers.setLinearHeadingInterpolation(startPose.getHeading(), -90, .8);
        fromStartToChambers.setConstantHeadingInterpolation(startPose.getHeading());
        fromStartToChambers.setPathEndTimeoutConstraint(3);

        telemetry.addData("StartToChambers End Tangent: ", MathFunctions.radToDeg(fromStartToChambers.getEndTangent().getTheta()));

        fromChambersToObservation = new EasyPath(fromStartToChambers.getLastControlPoint(), poses.Observations.Blue, new double[]{0}, new double[]{-vars.Chassis.FRONT_LENGTH});
        //Same as above
//        fromChambersToObservation.setLinearHeadingInterpolation(MathFunctions.degToRad(-90), MathFunctions.degToRad(90), .8);
        fromChambersToObservation.setLinearHeadingInterpolation(new SafeInterpolationStartHeading(fromStartToChambers.getEndTangent().getTheta(), poses.Observations.Blue).getValue(), poses.Observations.Blue.getHeading(), .8);
        fromChambersToObservation.setPathEndTimeoutConstraint(3);

        telemetry.addData("Blue Chamber: ", poses.Chambers.Blue);
        telemetry.addData("Blue Observation: ", poses.Observations.Blue);
        telemetry.update();
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
                if (pathTimer.getElapsedTime() > 2000) {
                    setPathState(14);
                }
            case 14:
                follower.followPath(fromChambersToObservation);
                setPathState(15);
                break;
            case 15:
                if (!follower.isBusy()) {
//                    follower.holdPoint(new BezierPoint(fromChambersToObservation.getLastControlPoint()), fromChambersToObservation.getEndTangent().getTheta());
//                    telemetry.addLine("Complete!");
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
