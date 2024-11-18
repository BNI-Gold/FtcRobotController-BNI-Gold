package org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Controls.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.BotPose.Pinpoint;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.BotPose.PoseHelper;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.BotPose.Vision;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Mechanisms.Intake.Intake;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Mechanisms.Intake.IntakeDirections;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Mechanisms.PrimaryArm.PrimaryArm;
import org.firstinspires.ftc.teamcode.Competition.IntoTheDeep.Gold10219.Robots.CompBot.CompBot;

public abstract class AutoMain extends LinearOpMode {

    // Constructor for the Competition Robot Class
    public CompBot Bot = new CompBot();

//    public PrimaryArm arm = new PrimaryArm();
//
//    public Intake intake = new Intake();

    public Vision vision = new Vision();
    public Pinpoint pinpoint = new Pinpoint();
    public PoseHelper pose = new PoseHelper();

    // Initialization, LinearOp, and Telemetry Update for all Auto Paths
    public void autoStart(){
        Bot.initRobot(hardwareMap);
        Bot.setLinearOp(this);
//        arm.initPrimaryArm(hardwareMap, Bot.LinearOp);
//        intake.initIntake(hardwareMap);
//        intake.center();

        pinpoint.setLinearOp(this);
        pinpoint.initPinpoint(hardwareMap);

        vision.setLinearOp(this);
        vision.initVision(hardwareMap, pinpoint, false, 4, "automain_");

        pose.setLinearOp(this);
        pose.setDevices(vision, pinpoint);

        telemetry.addLine("Awaiting Start");
        telemetry.update();
    }

    //Helper Method to Raise, Extend, and finish Raising Arm
    public void safeExtendAndRaise() {
//        arm.up(3, false);
//        sleep(500);
//        //stop for 1000ms=1sec
//        arm.extend(.75, 5);
//        sleep(500);
//        arm.up(5.6, true);
    }

    // Helper Method to Drop Sample in Net Area
    public void dropAndRetreatFromBucket() {
        // Must be called once intake is positioned at top bucket.
        // After dropping, will rotate, back, then strafe robot to be against wall after dropping, and retract arm. Arm stays up.
//        intake.start(IntakeDirections.OUT);
//        sleep(1000);
//        intake.stop();
//        sleep(500);
//        Bot.driveBackInches(0.5, 6);
//        sleep(500);
//        Bot.rotateRightDegrees(0.5, 45);
//        sleep(500);
//        Bot.strafeLeftInches(0.5, 15);
//        sleep(500);
//        Bot.driveBackInches(0.5, 4.8);
//        sleep(500);
//        arm.retract(.75, 5.1);
    }
}
